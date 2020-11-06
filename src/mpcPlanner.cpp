#include "mpcPlanner.h"

extern "C" {
extern void simplempc_casadi2forces( simplempc_float *x,        /* primal vars                                         */
                                                 simplempc_float *y,        /* eq. constraint multiplers                           */
                                                 simplempc_float *l,        /* ineq. constraint multipliers                        */
                                                 simplempc_float *p,        /* parameters                                          */
                                                 simplempc_float *f,        /* objective function (scalar)                         */
                                                 simplempc_float *nabla_f,  /* gradient of objective function                      */
                                                 simplempc_float *c,        /* dynamics                                            */
                                                 simplempc_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                                 simplempc_float *h,        /* inequality constraints                              */
                                                 simplempc_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                                 simplempc_float *hess,     /* Hessian (column major)                              */
                                                 solver_int32_default stage,     /* stage number (0 indexed)                            */
                                                 solver_int32_default iteration, /* iteration number of solver                          */
                                                 solver_int32_default threadID /* iteration number of solver                          */);
simplempc_extfunc extfunc_eval = &simplempc_casadi2forces;
}

// Constructor
MpcPlanner::MpcPlanner(std::string name) :
  as_(nh_, name, boost::bind(&MpcPlanner::executeCB, this, _1), false),
  action_name_(name),
  frequency_(5.0)
{
  // Setting variables and vector length
  ROS_INFO("Setting parameters from parameter space");
  nh_.getParam("/mpc/frequency", frequency_);
  nh_.getParam("/mpc/dt1", dt1_);
  nh_.getParam("/mpc/dt2", dt2_);
  nh_.getParam("/mpc/timeHorizon", timeHorizon_);
  rate_ = new ros::Rate(frequency_);
  dumpedProblems_ = 0;
  ROS_INFO("TIME HORIZON SET TO %d", timeHorizon_);
  globalPath_.resize(timeHorizon_);
  nh_.getParam("/mpc/velRedWheels", velRedWheels_);
  nh_.getParam("/mpc/velRedArm", velRedArm_);
  getMotionParameters("navigation");
  setConstantParameters();
  ROS_INFO("Initializing subscribers and publisher");
  // Service Client
  makePlanClient_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", true);
  
  // Publisher and Subscriber
  pubRightWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/right_wheel/command", 10);
  pubLeftWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/left_wheel/command", 10);
  pubArm_ = nh_.advertise<std_msgs::Float64MultiArray>("/mmrobot/multijoint_command", 10);
  pubPredTraj_ = nh_.advertise<nav_msgs::Path>("/mpc/predicted_trajectory", 10);
  subJointPosition_ = nh_.subscribe("/mmrobot/curState", 10, &MpcPlanner::state_cb, this);
  subConstraints_base1_ = nh_.subscribe("/constraints_base1", 10, &MpcPlanner::constraints_base1_cb, this);
  subConstraints_base2_ = nh_.subscribe("/constraints_base2", 10, &MpcPlanner::constraints_base2_cb, this);
  subConstraints_mid_ = nh_.subscribe("/constraints_mid", 10, &MpcPlanner::constraints_mid_cb, this);
  subConstraints_ee_ = nh_.subscribe("/constraints_ee", 10, &MpcPlanner::constraints_ee_cb, this);
  subGlobalPath_ = nh_.subscribe("/spline/globalPath", 10, &MpcPlanner::globalPath_cb, this);
  pubPredTraj_ = nh_.advertise<nav_msgs::Path>("/mpc/predicted_trajectory", 10);
  pubPredTrajArm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpc/predicted_traj_arm", 10);
  pubPredMove_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/mpc/jointTrajectory", 10);
  pubSolverInfo_ = nh_.advertise<mm_msgs::SolverInfo>("/mpc/solver_info", 10);
  subMovingObstacles_ = nh_.subscribe("/moving_obstacle", 10, &MpcPlanner::movingObstacles_cb, this);
  //subMovingObstacles_ = nh_.subscribe("/moving_obstacle2", 10, &MpcPlanner::movingObstacles2_cb, this);
  subResetDumpNumber_ = nh_.subscribe("/mpc/resetDump", 10, &MpcPlanner::resetDumpNumber_cb, this);
  finalBaseGoal_.fill(0.0);
  initializePlanes();
  initializeMovingObstacles();
  as_.start();
  ros::spinOnce();
  ROS_INFO("INITIALIZED MPC SERVER");
}

// Start: 
// ******CALLBACKS******
//
void MpcPlanner::globalPath_cb(const mm_msgs::NurbsEval2D::ConstPtr& evalNurbs)
{
  //ROS_INFO("RECEIVED PLAN EVALUATION");
  for (unsigned int i = 0; i < timeHorizon_; ++i) {
    globalPath_[i][0] = evalNurbs->evaluations[i].x;
    globalPath_[i][1] = evalNurbs->evaluations[i].y;
    if (isCloseToTarget()) globalPath_[i][2] = finalBaseGoal_[2];
    else globalPath_[i][2] = evalNurbs->evaluations[i].theta;
  }
}

void MpcPlanner::movingObstacles_cb(const mm_msgs::DynamicObstacleMsg::ConstPtr& data)
{
  movingObstacles_[0] =  data->pose.position.x;
  movingObstacles_[1] =  data->pose.position.y;
  movingObstacles_[2] =  data->pose.position.z;
  movingObstacles_[3] =  data->twist.linear.x;
  movingObstacles_[4] =  data->twist.linear.y;
  movingObstacles_[5] =  data->twist.linear.z;
  movingObstacles_[6] =  data->size.data;
}

bool MpcPlanner::isCloseToTarget()
{
  unsigned int i;
  double baseError = 0.0;
  for (i = 0; i < 2; ++i) {
    baseError += mType_.errorWeights[0] * pow((curState_[i] - finalBaseGoal_[i]), 2);
  }
  return baseError < 1.0 * mType_.accuracy;
}

void MpcPlanner::state_cb(const sensor_msgs::JointState::ConstPtr& data)
{
  //ROS_INFO("Receiving Joint States");
  // Getting state information
  for (int i = 0; i < 10; ++i) {
    curState_[i] = data->position[i];
  }
  // Getting velocity information wheels
  for (int i = 0; i < 2; ++i) {
    curU_[i] = data->velocity[i+10];
  }
  // Getting velocity information joints
  for (int i = 0; i < 7; ++i) {
    curU_[i+2] = data->velocity[i+3];
  }
}

void MpcPlanner::constraints_base1_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  for (int i = 0; i < NIPES; ++i)
  {
    if (i < data->constraints.size()) {
      planesBase1_[4 * i + 0] = -1 * data->constraints[i].A[0];
      planesBase1_[4 * i + 1] = -1 * data->constraints[i].A[1];
      planesBase1_[4 * i + 2] = -1 * data->constraints[i].A[2];
      planesBase1_[4 * i + 3] = -1 * data->constraints[i].b;
    }
    else {
      planesBase1_[4 * i + 0] = 0;
      planesBase1_[4 * i + 1] = 0;
      planesBase1_[4 * i + 2] = 1;
      planesBase1_[4 * i + 3] = -100;
    }
  }
}

void MpcPlanner::constraints_base2_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    if (i < data->constraints.size()) {
      planesBase2_[4 * i + 0] = -1 * data->constraints[i].A[0];
      planesBase2_[4 * i + 1] = -1 * data->constraints[i].A[1];
      planesBase2_[4 * i + 2] = -1 * data->constraints[i].A[2];
      planesBase2_[4 * i + 3] = -1 * data->constraints[i].b;
    }
    else {
      planesBase2_[4 * i + 0] = 0;
      planesBase2_[4 * i + 1] = 0;
      planesBase2_[4 * i + 2] = 1;
      planesBase2_[4 * i + 3] = -100;
    }
  }
}

void MpcPlanner::constraints_mid_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    if (i < data->constraints.size()) {
      planesMid_[4 * i + 0] = -1 * data->constraints[i].A[0];
      planesMid_[4 * i + 1] = -1 * data->constraints[i].A[1];
      planesMid_[4 * i + 2] = -1 * data->constraints[i].A[2];
      planesMid_[4 * i + 3] = -1 * data->constraints[i].b;
    }
    else {
      planesMid_[4 * i + 0] = 0;
      planesMid_[4 * i + 1] = 0;
      planesMid_[4 * i + 2] = 1;
      planesMid_[4 * i + 3] = -100;
    }
  }
}

void MpcPlanner::constraints_ee_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    if (i < data->constraints.size()) {
      planesEE_[4 * i + 0] = -1 * data->constraints[i].A[0];
      planesEE_[4 * i + 1] = -1 * data->constraints[i].A[1];
      planesEE_[4 * i + 2] = -1 * data->constraints[i].A[2];
      planesEE_[4 * i + 3] = -1 * data->constraints[i].b;
    }
    else {
      planesEE_[4 * i + 0] = 0;
      planesEE_[4 * i + 1] = 0;
      planesEE_[4 * i + 2] = 1;
      planesEE_[4 * i + 3] = -100;
    }
  }
}

void MpcPlanner::resetDumpNumber_cb(const std_msgs::Bool::ConstPtr& data)
{
  dumpedProblems_ = 0;
}
// End: 
// ******CALLBACKS******
//

// Start: 
// ******PARAMETER******
//
void MpcPlanner::initializePlanes()
{
  for (int i = 0; i < 15; ++i) {
    planesBase1_[4 * i + 0] = 0;
    planesBase1_[4 * i + 1] = 0;
    planesBase1_[4 * i + 2] = 1;
    planesBase1_[4 * i + 3] = -200;
    planesBase2_[4 * i + 0] = 0;
    planesBase2_[4 * i + 1] = 0;
    planesBase2_[4 * i + 2] = 1;
    planesBase2_[4 * i + 3] = -300;
    planesMid_[4 * i + 0] = 0;
    planesMid_[4 * i + 1] = 0;
    planesMid_[4 * i + 2] = 1;
    planesMid_[4 * i + 3] = -400;
    planesEE_[4 * i + 0] = 0;
    planesEE_[4 * i + 1] = 0;
    planesEE_[4 * i + 2] = 1;
    planesEE_[4 * i + 3] = -500;
  }
}

void MpcPlanner::initializeMovingObstacles()
{
  for (int i = 0; i < 5; ++i) {
    movingObstacles_[7 * i + 0] = 0.0;
    movingObstacles_[7 * i + 1] = 0.0;
    movingObstacles_[7 * i + 2] = 0.0;
    movingObstacles_[7 * i + 3] = 0.0;
    movingObstacles_[7 * i + 4] = 0.0;
    movingObstacles_[7 * i + 5] = 0.0;
    movingObstacles_[7 * i + 6] = -50.0;
  }
}

void MpcPlanner::getMotionParameters(std::string motionId)
{
  //ROS_INFO("GETTING MOTION TYPE RELATED PARAMETERS"); 
  nh_.getParam("/mpc/maxError", mType_.accuracy);
  nh_.getParam("/mpc/" + motionId + "/safetyMarginBase", mType_.safetyMarginBase);
  nh_.getParam("/mpc/" + motionId + "/safetyMarginArm", mType_.safetyMarginArm);
  nh_.getParam("/mpc/" + motionId + "/weights", mType_.weights);
  nh_.getParam("/mpc/" + motionId + "/errorWeights", mType_.errorWeights);
}

void MpcPlanner::setForcesParams()
{
  // Setting xinit
  for (unsigned int i = 0; i < 10; ++i) {
    mpc_params_.xinit[i] = curState_[i];
  }
  //mpc_params_.xinit[10] = curSlack_;
  for (unsigned int i = 0; i < 9; ++i) {
    mpc_params_.xinit[10 + i] = curU_[i];
  }
  // Setting x0 and params for all timesteps
  for (unsigned int t = 0; t < 15; ++t) {
    // Setting xO
    for (unsigned int i = 0; i < 10; ++i) {
      mpc_params_.x0[i + t * 20] = curState_[i];
    }
    mpc_params_.x0[10 + t * 20] = curSlack_;
    for (unsigned int j = 11; j < 20; ++j) {
      mpc_params_.x0[j + t * 20] = curU_[j - 11];
    }
    // Setting params
    for (unsigned int p = 0; p < nbParams_; ++p) {
      mpc_params_.all_parameters[t * nbParams_ + p] = params_[p];
    }
  }
}

void MpcPlanner::setConstantParameters()
{
  params_[0] = dt1_;
  params_[1] = dt2_;
  nh_.getParam("/wheelRadius", params_[2]);
  nh_.getParam("/wheelSeperator", params_[3]);
}

void MpcPlanner::setChangingParameters()
{
  //ROS_INFO("SETTING CHANGING PARAMETERS");
  // Spline
  for (unsigned int t = 0; t < timeHorizon_; ++t) {
    params_[4 + t * 3 + 0] = globalPath_[t][0];
    params_[4 + t * 3 + 1] = globalPath_[t][1];
    params_[4 + t * 3 + 2] = globalPath_[t][2];
  }
  // Weights
  for (unsigned int i = 0; i < 7; ++i) {
    params_[57 + i] = mType_.weights[i];
  }
  // SafetyMargin
  params_[64] = mType_.safetyMarginBase;
  params_[65] = mType_.safetyMarginArm;
  // InfPlanes Base1
  for (unsigned int i = 0; i < 15 * 4; ++i) {
    params_[66 + i] = planesBase1_[i];
    params_[126 + i] = planesBase2_[i];
    params_[186 + i] = planesMid_[i];
    params_[246 + i] = planesEE_[i];
  }
  // Moving Obstacles
  for (unsigned int i = 0; i < 35; ++i) {
    params_[306 + i] = movingObstacles_[i];
  }
}

void MpcPlanner::clearVariables()
{
  params_.fill(0.0);
  setConstantParameters();
  initializePlanes();
  initializeMovingObstacles();
}

void MpcPlanner::resetInitialGuess()
{
  for (unsigned int t = 0; t < 15; ++t) {
    // Setting xO
    for (unsigned int i = 0; i < 10; ++i) {
      mpc_params_.x0[i + t * 20] = 0.0;
    }
    mpc_params_.x0[10 + t * 20] = 0.0;
    for (unsigned int j = 11; j < 20; ++j) {
      mpc_params_.x0[j + t * 20] = 0.0;
    }
  }
}

void MpcPlanner::setManualParameters()
{
  ROS_INFO("ONLY FOR DEBUGGING PURPOSE");
  params_[0] = 2.5;
  params_[1] = 0.08;
  params_[2] = 0.484;
  params_[3] = 0.25;
  params_[4] = 0.0;
  params_[5] = 0.0;
  params_[6] = 0.5;
  params_[7] = 0.0;
  params_[8] = 0.0;
  params_[9] = 0.75;
  params_[10] = 0.0;
  params_[11] = 0.0;
  params_[12] = 1.0;
  params_[13] = 0.0;
  params_[14] = 0.0;
  params_[15] = 1.25;
  params_[16] = 0.0;
  params_[17] = 0.0;
  params_[18] = 0.0;
  params_[19] = 1.0;
  params_[20] = 0.0;
  params_[21] = -1.5;
  params_[22] = -1.5;
  params_[23] = 0.5;
  params_[24] = 1.0;
  params_[25] = 0.2;
  params_[26] = 1.0;
  params_[27] = 1.0;
  params_[28] = 1.0;
  params_[29] = 0.0;
  params_[30] = 10.0;
  params_[31] = 0.5;
  params_[32] = 10.0;
  params_[33] = 0.5;
}

void MpcPlanner::setGoalParameters()
{
  // Arm Goal
  for (unsigned int c = 0; c < 7; ++c) {
    params_[50 + c] = armGoal_[c];
  }
  // Orientation Goal deprecated
  params_[49] = oGoal_;
}
// End: 
// ******PARAMETER******
//
// Start
// ******ACTION******
//
nav_msgs::GetPlan MpcPlanner::makePathRequest(const geometry_msgs::Pose2D baseGoal)
{
  //ROS_INFO("CREATING PATH REQUEST");
  nav_msgs::GetPlan pathPlan;
  std::string refFrame;
  nh_.getParam("/reference_frame", refFrame);
  pathPlan.request.start.header.frame_id = refFrame;
  pathPlan.request.start.pose.position.x = curState_[0];
  pathPlan.request.start.pose.position.y = curState_[1];
  pathPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(curState_[2]);
  pathPlan.request.goal.header.frame_id = refFrame;
  pathPlan.request.goal.pose.position.x = baseGoal.x;
  pathPlan.request.goal.pose.position.y = baseGoal.y;
  pathPlan.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(baseGoal.theta);
  // TODO Should be in line with the tolerance for the MPC
  pathPlan.request.tolerance = 0.01;
  return pathPlan;
}

void MpcPlanner::updatePathRequest()
{
  ROS_INFO("COMPUTING NEW GLOBAL PLAN");
  myPath_.request.start.pose.position.x = curState_[0];
  myPath_.request.start.pose.position.y = curState_[1];
  myPath_.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(curState_[2]);
}

int MpcPlanner::solve()
{
  int exitFlag = simplempc_solve(&mpc_params_, &mpc_output_, &mpc_info_, stdout, extfunc_eval);
  //if(exitFlag < 1) dumpProblem();
  mm_msgs::SolverInfo info;
  info.exitFlag = (int8_t)exitFlag;
  info.nbIterations = (int16_t)mpc_info_.it;
  info.solvingTime = (double)mpc_info_.solvetime;
  pubSolverInfo_.publish(info);
  return exitFlag;
}

double MpcPlanner::computeError()
{
  std::array<double, 10> jointErrors;
  unsigned int i;
  for (i = 0; i < 2; ++i) {
    jointErrors[i] = mType_.errorWeights[0] * pow((curState_[i] - finalBaseGoal_[i]), 2);
  }
  for (i = 2; i < 3; ++i) {
    jointErrors[i] = mType_.errorWeights[1] * pow((curState_[i] - finalBaseGoal_[i]), 2);
  }
  int armIndex = 0;
  for (i = 3; i < 10; ++i) {
    armIndex = i - 3;
    jointErrors[i] = mType_.errorWeights[2] * pow((curState_[i] - armGoal_[armIndex]), 2);
  }
  int index = std::distance(jointErrors.begin(), std::max_element(jointErrors.begin(), jointErrors.end()));
  double largestErrorContribution = jointErrors[index];
  double totError = 0.0;
  for (int i = 0; i < 10; ++i) {
    totError += jointErrors[i];
  }
  ROS_INFO("Max Error of %1.3f at %d", sqrt(largestErrorContribution), index);
  double curError = sqrt(totError);
  ROS_INFO("Current summed error is %1.3f", curError);
  return curError;
}

void MpcPlanner::publishVelocities(std::array<double, 9> vel)
{
  //ROS_INFO("Publish Velocities");
  std_msgs::Float64 right_vel;
  std_msgs::Float64 left_vel;
  left_vel.data = vel[0];
  right_vel.data = vel[1];
  pubLeftWheel_.publish(left_vel);
  pubRightWheel_.publish(right_vel);
  //ROS_INFO("Right_vel %1.3f and left_vel %1.3f", right_vel.data, left_vel.data);
  std_msgs::Float64MultiArray arm_vel;
  for (int i = 0; i < 7; ++i) {
    arm_vel.data.push_back(vel[i+2]);
  }
  pubArm_.publish(arm_vel);
  //ROS_INFO("Published Velocities");
}

void MpcPlanner::publishZeroVelocities()
{
  std::array<double, 9> zeroVel = {0, 0, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001};
  publishVelocities(zeroVel);
}

void MpcPlanner::processOutput(int exitFlag)
{
  curSlack_ = mpc_output_.x02[10];
  if (exitFlag > 0) {
    std::array<double, 9> vel;
    for (int i = 0; i < 9; ++i) {
      vel[i] = mpc_output_.x02[i + 11];  
    }
    publishVelocities(vel);
    shiftTime();
    publishPredTraj();
    rate_->sleep();
    errorFlagCounter_ = 0;
  }
  else {
    errorFlagCounter_++;
    std::array<double, 9> vel;
    //ROS_INFO("VEL REDUCTION OF %1.5f AND %1.5f", velRedWheels_, velRedArm_);
    for (int i = 0; i < 2; ++i) {
      // 0.2
      vel[i] = velRedWheels_ * mpc_output_.x02[i + 11];  
    }
    for (int i = 2; i < 9; ++i) {
      // 0.1
      vel[i] = velRedArm_ * mpc_output_.x02[i + 11];  
    }
    //printParams();
    publishVelocities(vel);
    /*
    if (!isCloseToTarget()) {
      updatePathRequest();
      makePlanClient_.call(myPath_);
    }
    */
    //publishPredTraj();
    if (errorFlagCounter_ > 10) {
      clearVariables();
    }
    setGoalParameters();
    setChangingParameters();
    rate_->sleep();
  }
}

void MpcPlanner::publishPredTraj()
{
  //ROS_INFO("Publishing predicted trajectory");
  trajectory_msgs::JointTrajectory pred_arm_traj_msg;
  trajectory_msgs::MultiDOFJointTrajectory pred_base_msg;
  nav_msgs::Path pred_traj_msg;
  pred_traj_msg.poses.resize(13);
  pred_arm_traj_msg.points.resize(13);
  pred_base_msg.points.resize(13);
  std::string refFrame;
  nh_.getParam("/reference_frame", refFrame);
  pred_traj_msg.header.frame_id = refFrame;
  pred_arm_traj_msg.header.frame_id = refFrame;
  pred_base_msg.header.frame_id = refFrame;
  pred_arm_traj_msg.joint_names = {"mmrobot_joint1", "mmrobot_joint2", "mmrobot_joint3","mmrobot_joint4", "mmrobot_joint5", "mmrobot_joint6","mmrobot_joint7"};
  pred_base_msg.joint_names = {"base_link"};
  for (int i = 0; i < 13; ++i) {
    pred_base_msg.points[i].transforms.resize(1);
    pred_traj_msg.poses[i].header.frame_id = refFrame;
    pred_arm_traj_msg.points[i].positions.resize(7);
    pred_arm_traj_msg.points[i].time_from_start = ros::Duration(0.2 * i);
  }
  pred_base_msg.points[0].transforms[0].translation.x = mpc_output_.x02[0];
  pred_base_msg.points[1].transforms[0].translation.x = mpc_output_.x03[0];
  pred_base_msg.points[2].transforms[0].translation.x = mpc_output_.x04[0];
  pred_base_msg.points[3].transforms[0].translation.x = mpc_output_.x05[0];
  pred_base_msg.points[4].transforms[0].translation.x = mpc_output_.x06[0];
  pred_base_msg.points[5].transforms[0].translation.x = mpc_output_.x07[0];
  pred_base_msg.points[6].transforms[0].translation.x = mpc_output_.x08[0];
  pred_base_msg.points[7].transforms[0].translation.x = mpc_output_.x09[0];
  pred_base_msg.points[8].transforms[0].translation.x = mpc_output_.x10[0];
  pred_base_msg.points[9].transforms[0].translation.x = mpc_output_.x11[0];
  pred_base_msg.points[10].transforms[0].translation.x = mpc_output_.x12[0];
  pred_base_msg.points[11].transforms[0].translation.x = mpc_output_.x13[0];
  pred_base_msg.points[12].transforms[0].translation.x = mpc_output_.x14[0];
  pred_base_msg.points[0].transforms[0].translation.y = mpc_output_.x02[0];
  pred_base_msg.points[1].transforms[0].translation.y = mpc_output_.x03[0];
  pred_base_msg.points[2].transforms[0].translation.y = mpc_output_.x04[0];
  pred_base_msg.points[3].transforms[0].translation.y = mpc_output_.x05[0];
  pred_base_msg.points[4].transforms[0].translation.y = mpc_output_.x06[0];
  pred_base_msg.points[5].transforms[0].translation.y = mpc_output_.x07[0];
  pred_base_msg.points[6].transforms[0].translation.y = mpc_output_.x08[0];
  pred_base_msg.points[7].transforms[0].translation.y = mpc_output_.x09[0];
  pred_base_msg.points[8].transforms[0].translation.y = mpc_output_.x10[0];
  pred_base_msg.points[9].transforms[0].translation.y = mpc_output_.x11[0];
  pred_base_msg.points[10].transforms[0].translation.y = mpc_output_.x12[0];
  pred_base_msg.points[11].transforms[0].translation.y = mpc_output_.x13[0];
  pred_base_msg.points[12].transforms[0].translation.y = mpc_output_.x14[0];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[0].positions[i] = mpc_output_.x02[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[1].positions[i] = mpc_output_.x03[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[2].positions[i] = mpc_output_.x04[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[3].positions[i] = mpc_output_.x05[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[4].positions[i] = mpc_output_.x06[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[5].positions[i] = mpc_output_.x07[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[6].positions[i] = mpc_output_.x08[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[7].positions[i] = mpc_output_.x09[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[8].positions[i] = mpc_output_.x10[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[9].positions[i] = mpc_output_.x11[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[10].positions[i] = mpc_output_.x12[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[11].positions[i] = mpc_output_.x13[3+i];
  for(int i = 0; i < 7; ++i) pred_arm_traj_msg.points[12].positions[i] = mpc_output_.x14[3+i];
  pred_traj_msg.poses[0].pose.position.x = mpc_output_.x02[0];
  pred_traj_msg.poses[0].pose.position.y = mpc_output_.x02[1];
  pred_traj_msg.poses[1].pose.position.x = mpc_output_.x03[0];
  pred_traj_msg.poses[1].pose.position.y = mpc_output_.x03[1];
  pred_traj_msg.poses[2].pose.position.x = mpc_output_.x04[0];
  pred_traj_msg.poses[2].pose.position.y = mpc_output_.x04[1];
  pred_traj_msg.poses[3].pose.position.x = mpc_output_.x05[0];
  pred_traj_msg.poses[3].pose.position.y = mpc_output_.x05[1];
  pred_traj_msg.poses[4].pose.position.x = mpc_output_.x06[0];
  pred_traj_msg.poses[4].pose.position.y = mpc_output_.x06[1];
  pred_traj_msg.poses[5].pose.position.x = mpc_output_.x07[0];
  pred_traj_msg.poses[5].pose.position.y = mpc_output_.x07[1];
  pred_traj_msg.poses[6].pose.position.x = mpc_output_.x08[0];
  pred_traj_msg.poses[6].pose.position.y = mpc_output_.x08[1];
  pred_traj_msg.poses[7].pose.position.x = mpc_output_.x09[0];
  pred_traj_msg.poses[7].pose.position.y = mpc_output_.x09[1];
  pred_traj_msg.poses[8].pose.position.x = mpc_output_.x10[0];
  pred_traj_msg.poses[8].pose.position.y = mpc_output_.x10[1];
  pred_traj_msg.poses[9].pose.position.x = mpc_output_.x11[0];
  pred_traj_msg.poses[9].pose.position.y = mpc_output_.x11[1];
  pred_traj_msg.poses[10].pose.position.x = mpc_output_.x12[0];
  pred_traj_msg.poses[10].pose.position.y = mpc_output_.x12[1];
  pred_traj_msg.poses[11].pose.position.x = mpc_output_.x13[0];
  pred_traj_msg.poses[11].pose.position.y = mpc_output_.x13[1];
  pred_traj_msg.poses[12].pose.position.x = mpc_output_.x14[0];
  pred_traj_msg.poses[12].pose.position.y = mpc_output_.x14[1];
  pubPredTraj_.publish(pred_traj_msg);
  pubPredTrajArm_.publish(pred_arm_traj_msg);
  moveit_msgs::DisplayTrajectory t;
  t.trajectory.resize(1);
  t.trajectory[0].joint_trajectory = pred_arm_traj_msg;
  //t.trajectory[0].multi_dof_joint_trajectory = pred_base_msg;
  pubPredMove_.publish(t);
}

void MpcPlanner::shiftTime()
{
  for (int i = 0; i < 20; ++i) {
    mpc_params_.x0[i] = mpc_output_.x02[i];
    mpc_params_.x0[i+20] = mpc_output_.x03[i];
    mpc_params_.x0[i+40] = mpc_output_.x04[i];
    mpc_params_.x0[i+60] = mpc_output_.x05[i];
    mpc_params_.x0[i+80] = mpc_output_.x06[i];
    mpc_params_.x0[i+100] = mpc_output_.x07[i];
    mpc_params_.x0[i+120] = mpc_output_.x08[i];
    mpc_params_.x0[i+140] = mpc_output_.x09[i];
    mpc_params_.x0[i+160] = mpc_output_.x10[i];
    mpc_params_.x0[i+180] = mpc_output_.x11[i];
    mpc_params_.x0[i+200] = mpc_output_.x12[i];
    mpc_params_.x0[i+220] = mpc_output_.x13[i];
    mpc_params_.x0[i+240] = mpc_output_.x14[i];
    mpc_params_.x0[i+260] = mpc_output_.x15[i];
    mpc_params_.x0[i+280] = mpc_output_.x15[i];
  }
}
 

void MpcPlanner::executeCB(const mobile_mpc::simpleMpcGoalConstPtr& goal)
{
  ROS_INFO("MPC ACTION RECEIVED NEW PLAN");
  ros::Time startTime_, endTime_;
  startTime_ = ros::Time::now();
  // Call Global Planner Base
  myPath_ = makePathRequest(goal->goalPose.basePose);
  
  makePlanClient_.call(myPath_);
  for (int i = 0; i < 7; ++i) {
    armGoal_[i]= goal->goalPose.armConfig.data[i];
  }
  finalBaseGoal_[0] = goal->goalPose.basePose.x;
  finalBaseGoal_[1] = goal->goalPose.basePose.y;
  finalBaseGoal_[2] = goal->goalPose.basePose.theta;
  oGoal_ = goal->goalPose.basePose.theta;
  rate_->sleep();
  setGoalParameters();
  getMotionParameters(goal->motionType);
  setChangingParameters();
  int counter = 0;
  int ef;
  while (computeError() > mType_.accuracy) {
    //if (counter %10 == 0) makePlanClient_.call(myPath);
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s is preempted", action_name_.c_str());
      as_.setPreempted();
      result_.success = false;
      publishZeroVelocities();
      clearVariables();
      return;
    }
    setChangingParameters();
    setForcesParams();
    // Set initial guess to all zeros
    //resetInitialGuess();
    //ros::spinOnce();
    ef = solve();
    processOutput(ef);
    feedback_.curError = computeError();
    as_.publishFeedback(feedback_);
    counter++;
  }
  //printParams();
  endTime_ = ros::Time::now(); 
  publishZeroVelocities();
  feedback_.errorFlag = 0;
  result_.success = true;
  ros::Duration temp = endTime_ - startTime_;
  result_.executionTime.data = temp;
  as_.setSucceeded(result_);
  clearVariables();
  
}
// Start: 
// ******INFO******
//
void MpcPlanner::printOutput()
{
  ROS_INFO("Printing output");
  for (int i = 0; i < 20; ++i) {
    std::cout << mpc_output_.x02[i] << std::endl;
  }

}
void MpcPlanner::printParams()
{
  ROS_INFO("Printing %d parameters", 240);
  for (int i = 64; i < 304; ++i) {
    std::cout << i << " : " << params_[i] << std::endl;
  }
  /*
  ROS_INFO("Printing forces params");
  for (int i = 0; i < 5085; ++i) {
    std::cout << i << " : " << mpc_params_.all_parameters[i] << std::endl;
  }
  for (int i = 0; i < 20; ++i) {
    std::cout << "x0[" << i << "] : " << mpc_params_.x0[i] << std::endl;
    std::cout << "xinit[" << i << "] : " << mpc_params_.x0[i] << std::endl;
  }
  */
}

std::string MpcPlanner::getCurrentTimeString()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", timeinfo);
  std::string res(buffer);
  return res + "_" + std::to_string(dumpedProblems_);
}

void MpcPlanner::dumpProblem()
{
  ROS_INFO("ATTEMPTING TO DUMP PROBLEM, nr : %d", dumpedProblems_);
  if (dumpedProblems_ >= 10) {
    return;
  }
  std::string curTime = getCurrentTimeString();
  std::string fileName = "/home/mspahn/catkin_clean/src/mobile_mpc/dumpedCppFiles/dump_" + curTime + ".csv";
  ROS_INFO("DUMPING PROBLEM TO FILE %s", fileName.c_str());
  std::ofstream dumpFile;
  dumpFile.open(fileName);
  for(unsigned int i = 0; i < 20; ++i) {
    dumpFile << mpc_params_.xinit[i] << "\n";
  }
  for (unsigned int i = 0; i < 300; ++i) {
    dumpFile << mpc_params_.x0[i] << "\n";
  }
  for (unsigned int i = 0; i < 5115; ++i) {
    dumpFile << mpc_params_.all_parameters[i] << "\n";
  }
  dumpFile.close();
  dumpedProblems_++;
}

void MpcPlanner::runNode() 
{
  while (ros::ok()){
    rate_->sleep();
    //setForcesParams();
    //printParams();
    ros::spinOnce();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simpleMpc_server");
  MpcPlanner mpcplanner("simpleMpc_server");
  ros::spin();
  //mpcplanner.runNode();
  return 0;
}



