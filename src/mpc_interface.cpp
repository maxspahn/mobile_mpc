#include "mpc_interface.h"

MpcInterface::MpcInterface(std::string name) :
  mpcProblem_(0.25, 0.20),
  mpcSolver_(),
  name_(name)
{
  pubRightWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/right_wheel/command", 10);
  pubLeftWheel_ = nh_.advertise<std_msgs::Float64>("/mmrobot/left_wheel/command", 10);
  pubArm_ = nh_.advertise<std_msgs::Float64MultiArray>("/mmrobot/multijoint_command", 10);
  subJointPosition_ = nh_.subscribe("/mmrobot/joint_states", 10, &MpcInterface::jointState_cb, this);
  subConstraints_ = nh_.subscribe("/constraints", 10, &MpcInterface::constraints_cb, this);
  curState_ = {0};
  curU_ = {0};
  problemSetup();
}

MpcInterface::~MpcInterface()
{
  ROS_INFO("Calling Destructor of mpc");
}

void MpcInterface::problemSetup()
{
  // w_x, w_o, w_q, w_u, w_qdot, w_slack
  weightArray weights = {150.0, 10.0, 1.0, 0.5, 20.0, 0.0};
  mpcProblem_.weights(weights);
  mpcProblem_.param(10, 0.08);
  mpcProblem_.param(11, 0.544);
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

void MpcInterface::constraints_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr& data)
{
  for (int i = 0; i < data->constraints.size() ; ++i)
  {
    mpcProblem_.infPlane(4 * (i - 1) + 0, -1 * data->constraints[i].A[0]);
    mpcProblem_.infPlane(4 * (i - 1) + 1, -1 * data->constraints[i].A[1]);
    mpcProblem_.infPlane(4 * (i - 1) + 2, -1 * data->constraints[i].A[2]);
    mpcProblem_.infPlane(4 * (i - 1) + 3, -1 * data->constraints[i].b);
  }
  /*
  std::cout << "Received new constraints" << std::endl;
  printf("Plane[0] : A = [%1.4f, %1.4f, %1.4f], b = %1.4f\n", mpcProblem_.infPlane(0), mpcProblem_.infPlane(1), mpcProblem_.infPlane(2), mpcProblem_.infPlane(3));
  */
}

void MpcInterface::setGoal(goalArray goal)
{
  mpcProblem_.goal(goal);
}

void MpcInterface::setObstacles(obstacleArray obstacles)
{
  mpcProblem_.obstacles(obstacles);
}

void MpcInterface::setPlanes(planeArray planes)
{
  mpcProblem_.planes(planes);
}

void MpcInterface::parseProblem(goalArray goal, weightArray weights, errorWeightArray errorWeights)
{
  mpcProblem_.goal(goal);
  mpcProblem_.weights(weights);
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
  curUArray optCommands = mpcSolver_.getOptimalControl();
  //ROS_INFO("U_opt : %1.2f, %1.2f", optCommands[0], optCommands[1]);
  curU_[0] = optCommands[0];
  curU_[1] = optCommands[1];
  curU_[9] = optCommands[9];
  return optCommands;
}

void MpcInterface::getState()
{
  tf::StampedTransform strans;
  try {
    tfListener.lookupTransform("odom", "base_link", ros::Time(0), strans);
    tf::Matrix3x3 rotMatrixFast = strans.getBasis();
    tf::Vector3 posBase = strans.getOrigin();
    double roll, pitch, yaw;
    rotMatrixFast.getRPY(roll, pitch, yaw);
    curState_[0] = posBase[0];
    curState_[1] = posBase[1];
    curState_[2] = yaw;
    
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
}

double MpcInterface::computeError()
{
  double accum = 0.0;
  goalArray g = mpcProblem_.goal();
  int i;
  for (i = 0; i < 2; ++i) {
    accum += errorWeights_[0] * pow((curState_[i] - g[i]), 2);
  }
  for (i = 2; i < 3; ++i) {
    accum += errorWeights_[1] * pow((curState_[i] - g[i]), 2);
  }
  for (i = 3; i < NX; ++i) {
    accum += errorWeights_[2] * pow((curState_[i] - g[i]), 2);
  }
  ROS_INFO("Cur Error : %1.2f", sqrt(accum));
  return sqrt(accum);
}
