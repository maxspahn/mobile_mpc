#ifndef MPC_INTERFACE_H
#define MPC_INTERFACE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"

#include "mpc_forces_solver.h"
#include <mm_msgs/LinearConstraint3DArray.h>

#include <cmath>

typedef std::array<double, 3> errorWeightArray;

class MpcInterface
{
public:
  MpcInterface (std::string);
  virtual ~MpcInterface ();
  void problemSetup();
  void publishVelocities(curUArray vel);
  void publishZeroVelocities();
  void jointState_cb(const sensor_msgs::JointState::ConstPtr&);
  void constraints_base_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void constraints_mid_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void constraints_ee_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void printState();
  void setGoal(goalArray);
  void setObstacles(obstacleArray);
  void setPlanes(planeArray);
  void setInfPlanes(infPlaneArray);
  void parseProblem(goalArray, weightArray, errorWeightArray, double);
  curUArray solve();
  void getState();
  double computeError();

private:
  MpcProblem mpcProblem_;
  MpcForcesSolver mpcSolver_;
  std::string name_;
  ros::NodeHandle nh_;
  ros::Publisher pubRightWheel_;
  ros::Publisher pubLeftWheel_;
  ros::Publisher pubArm_;
  ros::Subscriber subJointPosition_;
  ros::Subscriber subConstraints_base_;
  ros::Subscriber subConstraints_mid_;
  ros::Subscriber subConstraints_ee_;
  tf::TransformListener tfListener;
  curStateArray curState_;
  curUArray curU_;
  errorWeightArray errorWeights_;
};

#endif /* MPC_INTERFACE_H */
