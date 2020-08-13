#include "mpc_interface.h"

MpcInterface::MpcInterface(std::string name) :
  mpcProblem_(0.5, 0.20),
  mpcSolver_(),
  name_(name)
{
  pubRightWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/right_wheel/command", 10);
  pubLeftWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/left_wheel/command", 10);
  pubArm_ = nh_.advertise<std_msgs::Float64MultiArray>("/mmrobot/multijoint_command", 10);
  pubSolveTime_ = nh_.advertise<std_msgs::Float64>("/solveTime", 10);
  subJointPosition_ = nh_.subscribe("/mmrobot/joint_states", 10, &MpcInterface::jointState_cb, this);
  subConstraints_base1_ = nh_.subscribe("/constraints_base1", 10, &MpcInterface::constraints_base1_cb, this);
  subConstraints_base2_ = nh_.subscribe("/constraints_base2", 10, &MpcInterface::constraints_base2_cb, this);
  subConstraints_mid_ = nh_.subscribe("/constraints_mid", 10, &MpcInterface::constraints_mid_cb, this);
  subConstraints_ee_ = nh_.subscribe("/constraints_ee", 10, &MpcInterface::constraints_ee_cb, this);
  curState_ = {0};
  curU_ = {0};
  problemSetup();
  nh_.getParam("/reference_frame", reference_frame_);
}

MpcInterface::~MpcInterface()
{
  ROS_INFO("Calling Destructor of mpc");
}

void MpcInterface::problemSetup()
{
  // w_x, w_o, w_q, w_u, w_qdot, w_slack
  weightArray weights = {6.0, 0.5, 1.0, 0.5, 20.0, 1000.0};
  mpcProblem_.weights(weights);
  mpcProblem_.param(10, 0.08);
  mpcProblem_.param(11, 0.544);
}

double MpcInterface::getRate()
{
  return 1.0/mpcProblem_.timeStep();
}

void MpcInterface::publishVelocities(curUArray vel)
{
  //ROS_INFO("Publish Velocities");
  std_msgs::Float64 right_vel;
  std_msgs::Float64 left_vel;
  right_vel.data = vel[1];
  left_vel.data = vel[0];
  pubRightWheel_.publish(right_vel);
  pubLeftWheel_.publish(left_vel);
  //ROS_INFO("Right_vel %1.3f and left_vel %1.3f", right_vel.data, left_vel.data);
  std_msgs::Float64MultiArray arm_vel;
  for (int i = 0; i < 7; ++i) {
    arm_vel.data.push_back(vel[i+2]);
  }
  pubArm_.publish(arm_vel);
  //ROS_INFO("Published Velocities");
}

void MpcInterface::publishZeroVelocities()
{
  curUArray zeroVel = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  publishVelocities(zeroVel);
}

void MpcInterface::jointState_cb(const sensor_msgs::JointState::ConstPtr& data)
{
  //ROS_INFO("Attempting joint state receive");
  for (int i = 0; i < 7; ++i) {
    curState_[i+3] = data->position[i+3];
    curU_[i+2] = data->velocity[i+3];
  }
  for (int i = 0; i < 2; ++i) {
    //curU_[i] = data->velocity[i+10];
  }
}

void MpcInterface::constraints_base1_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  //std::cout << "Receiving Constraints" << std::endl;
  for (int i = 0; i < NIPES; ++i) {
    mpcProblem_.infPlane(4 * i + 0, 0);
    mpcProblem_.infPlane(4 * i + 1, 0);
    mpcProblem_.infPlane(4 * i + 2, 1);
    mpcProblem_.infPlane(4 * i + 3, -10);
  }
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    //printf("Constraint received : %1.2fx + %1.2fy + %1.2fz = %1.2f\n", data->constraints[i].A[0], data->constraints[i].A[1], data->constraints[i].A[2], data->constraints[i].b);
    mpcProblem_.infPlane(4 * i + 0, -1 * data->constraints[i].A[0]);
    mpcProblem_.infPlane(4 * i + 1, -1 * data->constraints[i].A[1]);
    mpcProblem_.infPlane(4 * i + 2, -1 * data->constraints[i].A[2]);
    mpcProblem_.infPlane(4 * i + 3, -1 * data->constraints[i].b);
  }
  /*
  std::cout << "Received new constraints" << std::endl;
  printf("Plane[0] : A = [%1.4f, %1.4f, %1.4f], b = %1.4f\n", mpcProblem_.infPlane(0), mpcProblem_.infPlane(1), mpcProblem_.infPlane(2), mpcProblem_.infPlane(3));
  */
}

void MpcInterface::constraints_base2_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  int offset = 4 * NIPES;
  for (int i = 0; i < NIPES; ++i) {
    mpcProblem_.infPlane(4 * i + 0 + offset, 0);
    mpcProblem_.infPlane(4 * i + 1 + offset, 0);
    mpcProblem_.infPlane(4 * i + 2 + offset, 1);
    mpcProblem_.infPlane(4 * i + 3 + offset, -10);
  }
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    mpcProblem_.infPlane(4 * i + 0 + offset, -1 * data->constraints[i].A[0]);
    mpcProblem_.infPlane(4 * i + 1 + offset, -1 * data->constraints[i].A[1]);
    mpcProblem_.infPlane(4 * i + 2 + offset, -1 * data->constraints[i].A[2]);
    mpcProblem_.infPlane(4 * i + 3 + offset, -1 * data->constraints[i].b);
  }
}

void MpcInterface::constraints_mid_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  int offset = 4 * NIPES * 2;
  for (int i = 0; i < NIPES; ++i) {
    mpcProblem_.infPlane(4 * i + 0 + offset, 0);
    mpcProblem_.infPlane(4 * i + 1 + offset, 0);
    mpcProblem_.infPlane(4 * i + 2 + offset, 1);
    mpcProblem_.infPlane(4 * i + 3 + offset, -10);
  }
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    mpcProblem_.infPlane(4 * i + 0 + offset, -1 * data->constraints[i].A[0]);
    mpcProblem_.infPlane(4 * i + 1 + offset, -1 * data->constraints[i].A[1]);
    mpcProblem_.infPlane(4 * i + 2 + offset, -1 * data->constraints[i].A[2]);
    mpcProblem_.infPlane(4 * i + 3 + offset, -1 * data->constraints[i].b);
  }
}

void MpcInterface::constraints_ee_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  int offset = 4 * NIPES * 3;
  for (int i = 0; i < NIPES; ++i) {
    mpcProblem_.infPlane(4 * i + 0 + offset, 0);
    mpcProblem_.infPlane(4 * i + 1 + offset, 0);
    mpcProblem_.infPlane(4 * i + 2 + offset, 1);
    mpcProblem_.infPlane(4 * i + 3 + offset, -10);
  }
  for (int i = 0; i < data->constraints.size() && i < NIPES; ++i)
  {
    mpcProblem_.infPlane(4 * i + 0 + offset, -1 * data->constraints[i].A[0]);
    mpcProblem_.infPlane(4 * i + 1 + offset, -1 * data->constraints[i].A[1]);
    mpcProblem_.infPlane(4 * i + 2 + offset, -1 * data->constraints[i].A[2]);
    mpcProblem_.infPlane(4 * i + 3 + offset, -1 * data->constraints[i].b);
  }
}

void MpcInterface::setGoal(goalArray goal)
{
  mpcProblem_.goal(goal);
}

void MpcInterface::setObstacles(int limit, obstacleArray obstacles)
{
  mpcProblem_.obstacles(limit, obstacles);
}

void MpcInterface::setPlanes(planeArray planes)
{
  mpcProblem_.planes(planes);
}

void MpcInterface::setInfPlanes(infPlaneArray infPlanes)
{
  mpcProblem_.infPlanes(infPlanes);
}

void MpcInterface::parseProblem(goalArray goal, weightArray weights, errorWeightArray errorWeights, double safetyMargin)
{
  mpcProblem_.goal(goal);
  mpcProblem_.weights(weights);
  mpcProblem_.safetyMargin(safetyMargin);
  errorWeights_ = errorWeights;
  
}
  

void MpcInterface::printState()
{
  ROS_INFO("Current State");
  for (int i = 0; i < NX; ++i) {
    ROS_INFO("CurState[%d] : %1.4f", i, curState_[i]);
  }
}

curUArray MpcInterface::solve()
{
  //ROS_INFO("Start solve");
  getState();
  mpcProblem_.curU(curU_);
  mpcProblem_.curState(curState_);
  mpcSolver_.setupMPC(mpcProblem_);
  mpcSolver_.solveMPC();
  std_msgs::Float64 solveTime;
  solveTime.data = mpcSolver_.getSolveTime();
  pubSolveTime_.publish(solveTime);
  writeResultFile();
  int curExitFlag = mpcSolver_.getCurExitFlag();
  
  curUArray optCommands = mpcSolver_.getOptimalControl();
  //ROS_INFO("U_opt : %1.2f, %1.2f", optCommands[0], optCommands[1]);
  curU_[0] = optCommands[0];
  curU_[1] = optCommands[1];
  curU_[9] = optCommands[9];
  return optCommands;
}

void MpcInterface::writeResultFile() 
{
  outputFile_.open("/home/mspahn/test.txt", std::ios_base::app);
  outputFile_ << mpcSolver_.getCurExitFlag() << " ; " << mpcSolver_.getNbIter() << " ; " << mpcSolver_.getSolveTime() << "\n";
  outputFile_.close();
}

void MpcInterface::getState()
{
  tf::StampedTransform strans;
  try {
    tfListener.lookupTransform(reference_frame_, "base_link", ros::Time(0), strans);
    tf::Matrix3x3 rotMatrixFast = strans.getBasis();
    tf::Vector3 posBase = strans.getOrigin();
    double roll, pitch, yaw;
    rotMatrixFast.getRPY(roll, pitch, yaw);
    curState_[0] = posBase[0];
    curState_[1] = posBase[1];
    curState_[2] = yaw;
    
  }
  catch (tf::TransformException ex) {
    ROS_INFO("ERROR IN INTERFACE WITH TF");
    ROS_ERROR("%s", ex.what());
  }
}

double MpcInterface::computeError()
{
  double accum = 0.0;
  goalArray g = mpcProblem_.goal();
  double maxError = 0.0;
  double e;
  int maxErrorIndex = 0;
  int i;
  for (i = 0; i < 2; ++i) {
    e = errorWeights_[0] * pow((curState_[i] - g[i]), 2);
    if (e > maxError) {
      maxError = e;
      maxErrorIndex = i;
    }
    accum += e;
  }
  for (i = 2; i < 3; ++i) {
    e = errorWeights_[1] * pow((curState_[i] - g[i]), 2);
    if (e > maxError) {
      maxError = e;
      maxErrorIndex = i;
    }
    accum += e;
  }
  for (i = 3; i < NX; ++i) {
    e = errorWeights_[2] * pow((curState_[i] - g[i]), 2);
    if (e > maxError) {
      maxError = e;
      maxErrorIndex = i;
    }
    accum += e;
  }
  ROS_INFO("Max Error of %1.3f at %d\n", maxError, maxErrorIndex);
  return sqrt(accum);
}
