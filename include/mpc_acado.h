#ifndef MPC_ACADO_H
#define MPC_ACADO_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"

#include <acado_common.h>
#include <acado_auxiliary_functions.h>

#include <sstream>
#include <stdio.h>
#include <array>
#include <cmath>

class MpcAcado
{
private:
  std::string name;
  int time_horizon;
  ros::NodeHandle nh;
  ros::Publisher pubRightWheel;
  ros::Publisher pubLeftWheel;
  ros::Publisher pubArm;
  ros::Subscriber subJointPosition;
  ros::Rate rate;
  tf::TransformListener tfListener;
  int nbStateVar;
  int nbParams;
  int nbControl;
  int num_int;
  int num_steps;
  std::array<double, 11> curState;
  std::array<double, 10> curU;
  std::array<double, 10> goal;
  std::array<double, 18> params;
  double solverTolerance;

public:
  MpcAcado(std::string);
  ~MpcAcado();
  void runNode(std::array<double, 10>);
  void jointState_cb(const sensor_msgs::JointState::ConstPtr&);
  void getState();
  void singleMPCStep();
  std::array<double, 10> solve();
  void publishVelocities(std::array<double, 10>);
  void publishZeroVelocities();
  void problemSetup();
  void printState();
  void resetSolver();
  double computeError();
};

#endif /* MPC_ACADO_H */
    
