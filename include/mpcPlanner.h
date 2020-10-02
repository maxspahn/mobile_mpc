#ifndef MPCPLANNER_H
#define MPCPLANNER_H

#include "ros/ros.h"
// Messages
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "mm_msgs/LinearConstraint3DArray.h"
#include "mm_msgs/DynamicObstacleMsg.h"
#include "mm_msgs/NurbsEval2D.h"
// Action Server
#include <actionlib/server/simple_action_server.h>
#include <mobile_mpc/simpleMpcAction.h>

// TF
#include "tf/transform_datatypes.h"

// general c-staff
#include "cmath"
#include "fstream"
#include <numeric>

// Forces related staff
#include "simplempc.h"

#define NIPES 15

struct MotionType
{
  double accuracy;
  double threshholdCost;
  double safetyMargin;
  std::vector<double> weights;
  std::vector<double> errorWeights;
};

class MpcPlanner
{
protected:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  actionlib::SimpleActionServer<mobile_mpc::simpleMpcAction> as_;
  std::string action_name_;
  mobile_mpc::simpleMpcFeedback feedback_;
  mobile_mpc::simpleMpcResult result_;
  // Publisher
  ros::Publisher pubRightWheel_;
  ros::Publisher pubLeftWheel_;
  ros::Publisher pubArm_;
  //ros::Publisher pubSolveInfo_;
  ros::Publisher pubPredTraj_;
  // Subscriber
  ros::Subscriber subJointPosition_;
  ros::Subscriber subConstraints_base1_;
  ros::Subscriber subConstraints_base2_;
  ros::Subscriber subConstraints_mid_;
  ros::Subscriber subConstraints_ee_;
  ros::Subscriber subMovingObstacles_;
  ros::Subscriber subGlobalPath_;
  // Serrvices
  ros::ServiceClient makePlanClient_;
  // State variables
  std::array<double, 10> curState_;
  std::array<double, 9> curU_;
  std::array<double, 60> planesBase1_;
  std::array<double, 60> planesBase2_;
  std::array<double, 60> planesMid_;
  std::array<double, 60> planesEE_;
  std::array<double, 35> movingObstacles_;
  unsigned int nbParams_ = 339;
  std::array<double, 339> params_;
  unsigned int timeHorizon_;
  std::vector<std::array<double, 3>> globalPath_;
  std::array<double, 7> armGoal_;
  std::array<double, 3> finalBaseGoal_;
  double safetyMargin_;
  double oGoal_;
  double curSlack_;
  double dt_;
  nav_msgs::GetPlan myPath_;
  // Motion Type
  MotionType mType_;
  // Forces types
  simplempc_params mpc_params_;
  simplempc_output mpc_output_;
  simplempc_info mpc_info_;

public:
  MpcPlanner(std::string);
  // Callback functions
  void state_cb(const sensor_msgs::JointState::ConstPtr&);
  void constraints_base1_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void constraints_base2_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void constraints_mid_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void constraints_ee_cb(const mm_msgs::LinearConstraint3DArray::ConstPtr&);
  void globalPath_cb(const mm_msgs::NurbsEval2D::ConstPtr&);
  // Setting Getting Parameters
  void getMotionParameters(std::string);
  void initializePlanes();
  void initializeMovingObstacles();
  void setConstantParameters();
  void setChangingParameters();
  void setGoalParameters();
  void setForcesParams();
  void clearVariables();
  // Action
  nav_msgs::GetPlan makePathRequest(const geometry_msgs::Pose2D);
  void updatePathRequest();
  void executeCB(const mobile_mpc::simpleMpcGoalConstPtr&);
  int solve();
  void publishVelocities(std::array<double, 9>);
  void publishZeroVelocities();
  double computeError();
  bool isCloseToTarget();
  void processOutput(int);
  void publishPredTraj();
  void shiftTime();
  // Debug Section
  void setManualParameters();
  void printParams();
  void printOutput();
  // Running 
  void runNode();
};

#endif /* MPCPLANNER_H */