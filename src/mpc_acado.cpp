#include "mpc_acado.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MpcAcado::MpcAcado(std::string name) :
  rate(2),
  nbStateVar(ACADO_NX),
  nbControl(ACADO_NU),
  num_steps(100),
  num_int(ACADO_N),
  nbParams(18)
{
  name = name;

  pubRightWheel = nh.advertise<std_msgs::Float64>("/mmrobot/right_wheel/command", 10);
  pubLeftWheel = nh.advertise<std_msgs::Float64>("/mmrobot/left_wheel/command", 10);
  pubArm = nh.advertise<std_msgs::Float64MultiArray>("/mmrobot/multijoint_command", 10);
  subJointPosition = nh.subscribe("/mmrobot/joint_states", 10, &MpcAcado::jointState_cb, this);
  problemSetup();
  solverTolerance = 0.001;
  
}

MpcAcado::~MpcAcado()
{
  ROS_INFO("Calling Destructor of mpc");
}

void MpcAcado::runNode(std::array<double, 10> goal)
{
  rate.sleep();
  goal = goal;
  for (int j = 0; j < 10; ++j) {
    params[j] = goal[j];
  }
  resetSolver();
  while (computeError() > 0.1)
  //for (int i = 0; i < 30000; ++i)
  {
    //ROS_INFO("Attempting to get state");
    singleMPCStep();
    //ROS_INFO("Spinning cycle %d", i);
    ros::spinOnce();
  }
  publishZeroVelocities();
}

void MpcAcado::problemSetup()
{
  //ROS_INFO("Problem Setup");
  for (int i = 0; i < nbStateVar; ++i) {
    curState[i] = 0.0;
  }
  for (int i = 0; i < nbControl; ++i) {
    curU[i] = 0.0;
  }
  params[10] = 0.08;
  params[11] = 0.544;
  // w_x, w_o, w_q, w_u, w_qdot, w_slack
  //double weights[6] = {100.0, 0.0, 1.0, 0.5, 10.0, 0.0};
  double weights[6] = {150.0, 10.0, 1.0, 0.5, 20.0, 0.0};
  for (int i = 0; i < 6; ++i) {
    params[i+12] = weights[i];
  }
}

double MpcAcado::computeError()
{
  double accum = 0.0;
  for (int i = 0; i < curState.size(); ++i) {
    accum += pow((curState[i] - params[i]), 2);
  }
  ROS_INFO("Cur Error : %1.2f", sqrt(accum));
  return sqrt(accum);
}
  

void MpcAcado::publishVelocities(std::array<double, 10> vel)
{
  //ROS_INFO("Publish Velocities");
  std_msgs::Float64 right_vel;
  std_msgs::Float64 left_vel;
  right_vel.data = vel[1];
  left_vel.data = vel[0];
  pubRightWheel.publish(right_vel);
  pubLeftWheel.publish(left_vel);
  //ROS_INFO("Right_vel %1.3f and left_vel %1.3f", right_vel.data, left_vel.data);
  std_msgs::Float64MultiArray arm_vel;
  for (int i = 0; i < 7; ++i) {
    arm_vel.data.push_back(vel[i+2]);
  }
  pubArm.publish(arm_vel);
  //ROS_INFO("Published Velocities");
}

void MpcAcado::publishZeroVelocities()
{
  std::array<double, 10> zeroVel = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  publishVelocities(zeroVel);
}

void MpcAcado::jointState_cb(const sensor_msgs::JointState::ConstPtr& data)
{
  //ROS_INFO("Attempting joint state receive");
  for (int i = 0; i < 7; ++i) {
    curState[i+3] = data->position[i+3];
    curU[i+2] = data->velocity[i+3];
  }
  for (int i = 0; i < 2; ++i) {
    //curU[i] = data->velocity[i+10];
  }
}

void MpcAcado::singleMPCStep()
{
  //ROS_INFO("Single MPC Step");
  int exitFlag = 0;
  std::array<double, 10> u_opt = solve();
  for (int i = 0; i < 9; ++i) {
    //ROS_INFO("Opt Velocity[%d] : %1.4f", i, u_opt[i]);
  }
  rate.sleep();
  ROS_INFO("After Sleep");
  publishVelocities(u_opt);
  //ROS_INFO("Finished singleMPCStep");
}

void MpcAcado::printState()
{
  //ROS_INFO("Current State");
  for (int i = 0; i < nbStateVar; ++i) {
    //ROS_INFO("CurState[%d] : %1.4f", i, curState[i]);
  }
}

std::array<double, 10> MpcAcado::solve()
{
  //ROS_INFO("Start solve");
  getState();
  int i;
  acado_timer t;
  acado_initializeSolver();
  for (i = 0; i < nbStateVar * (num_int + 1); ++i)
  {
    // @todo Should be done using time horizon shift
    if (i < nbStateVar) acadoVariables.x[i] = curState[i];
    else acadoVariables.x[i] = 0.0;
  }
  //ROS_INFO("x set");
  for (i = 0; i < (nbControl * num_int); ++i)
  {
    //printf("curU[%d] : %1.2f\n", i % nbControl, curU[i % nbControl]);
    if (i < nbControl) acadoVariables.u[i] = curU[i];
    else acadoVariables.u[i] = 0.0;
  }
  //ROS_INFO("u set");
  for (i = 0; i < num_int; ++i)
  {
    for (int j = 0; j < nbParams; ++j)
    {
      //printf("Params[%d] : %1.2f\n", j, params[j]);
      acadoVariables.od[i * nbParams + j] = params[j];
    }
  }
  //ROS_INFO("OnlineData set");
  for (i = 0; i < nbStateVar; ++i)
  {
    //printf("curState[%d] : %1.2f\n", i, curState[i]);
    acadoVariables.x0[i] = curState[i];
  }
  //ROS_INFO("Attempting to solve MPC Problem");
	acado_preparationStep();
  acado_feedbackStep();
  int iter = 0;
  while (acado_getKKT() > solverTolerance && iter < num_steps)
  //while (iter < num_steps)
  {
    acado_feedbackStep();
	  acado_preparationStep();
    iter++;
    //ROS_INFO("Iteration Step : %d", iter);
    //ROS_INFO("KKT : %1.7f", acado_getKKT());
    //ROS_INFO("SOLVER TOL : %1.7f", solverTolerance);
    //ROS_INFO("U_opt : %1.2f, %1.2f", acadoVariables.u[0], acadoVariables.u[1]);
  }
  //ROS_INFO("Solved MPC Problem");
  //acado_printDifferentialVariables();
  //acado_printControlVariables();
  std::array<double, 10> optCommands;
  for (int i = 0; i < nbControl; ++i) {
    optCommands[i] = acadoVariables.u[i+10];
  }
  ROS_INFO("U_opt : %1.2f, %1.2f", optCommands[0], optCommands[1]);
  curU[0] = optCommands[0];
  curU[1] = optCommands[1];
  curU[9] = acadoVariables.u[9];
  return optCommands;
}

void MpcAcado::resetSolver()
{
  int i;
  for (i = 0; i < nbStateVar * (num_int + 1); ++i) acadoVariables.x[i] = 0.0;
  for (i = 0; i < (nbControl * num_int); ++i) acadoVariables.u[i] = 0.1;
  for (i = 0; i < num_int * nbParams; ++i) acadoVariables.od[i] = 0.1;
  for (i = 0; i < nbStateVar; ++i) acadoVariables.x0[i] = 0.0;
}

void MpcAcado::getState()
{
  tf::StampedTransform strans;
  try {
    tfListener.lookupTransform("odom", "base_link", ros::Time(0), strans);
    tf::Matrix3x3 rotMatrixFast = strans.getBasis();
    tf::Vector3 posBase = strans.getOrigin();
    double roll, pitch, yaw;
    rotMatrixFast.getRPY(roll, pitch, yaw);
    curState[0] = posBase[0];
    curState[1] = posBase[1];
    curState[2] = yaw;
    
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    //ros::Duration(1.0).sleep();
  }
}

  
  
/*
int main(int argc, char** argv)
{
  try
  {
    std::string name = "mpc_acado";
    ros::init(argc, argv, name);
    //double goal[10] = {2.0, 0.0, 0.0, 0.0, 0.5, 0.0, -1.0, 0.0, 2.1, 0.75};
    std::array<double, 10> goal = {0.0, 0.0, 0.0, 0.0, 0.5, 0.0, -1.0, 0.0, 2.1, 0.75};
    MpcAcado myMpc(name);
    myMpc.runNode(goal);
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Error in MPC: %s", e.what());
    exit(1);
  }
  return 0;
}
*/
