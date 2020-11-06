#ifndef MPCSPHEREPLANNER_H
#define MPCSPHEREPLANNER_H

#include "ros/ros.h"
// Messages
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Path.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "nav_msgs/GetPlan.h"
#include "mm_msgs/LinearConstraint3DArray.h"
#include "mm_msgs/DynamicObstacleMsg.h"
#include "mm_msgs/NurbsEval2D.h"
#include "mm_msgs/SolverInfo.h"
#include "mm_msgs/StaticSphereMsg.h"
// Action Server
#include <actionlib/server/simple_action_server.h>
#include <mobile_mpc/simpleMpcAction.h>

// TF
#include "tf/transform_datatypes.h"

// general c-staff
#include "cmath"
//#include "ctime"
#include "fstream"
#include <numeric>

// Forces related staff
#include "sphere5.h"

#define NSTATIC 5

struct MotionType
{
  double accuracy;
  double threshholdCost;
  double safetyMarginBase;
  double safetyMarginArm;
  std::vector<double> weights;
  std::vector<double> errorWeights;
};

class MpcSpherePlanner
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
  ros::Publisher pubPredTraj_;
  ros::Publisher pubPredTrajArm_;
  ros::Publisher pubSolverInfo_;
  ros::Publisher pubPredMove_;
  // Subscriber
  ros::Subscriber subJointPosition_;
  ros::Subscriber subStaticSpheres_;
  ros::Subscriber subMovingObstacles_;
  ros::Subscriber subGlobalPath_;
  ros::Subscriber subResetDumpNumber_;
  // Serrvices
  ros::ServiceClient makePlanClient_;
  // State variables
  std::array<double, 10> curState_;
  std::array<double, 9> curU_;
  std::array<double, 35> movingObstacles_;
  std::array<double, 4 * NSTATIC> staticSpheres_;
  unsigned int nbParams_ = 121;
  std::array<double, 121> params_;
  double velRedWheels_;
  double velRedArm_;
  unsigned int timeHorizon_;
  std::vector<std::array<double, 3>> globalPath_;
  std::array<double, 7> armGoal_;
  std::array<double, 3> finalBaseGoal_;
  double safetyMarginBase_;
  double safetyMarginArm_;
  double oGoal_;
  double curSlack_;
  double dt1_;
  double dt2_;
  unsigned int errorFlagCounter_;
  unsigned int dumpedProblems_;
  nav_msgs::GetPlan myPath_;
  // Motion Type
  MotionType mType_;
  // Forces types
  sphere5_params mpc_params_;
  sphere5_output mpc_output_;
  sphere5_info mpc_info_;

public:
  MpcSpherePlanner(std::string);
  // Callback functions
  void state_cb(const sensor_msgs::JointState::ConstPtr&);
  void globalPath_cb(const mm_msgs::NurbsEval2D::ConstPtr&);
  void movingObstacles_cb(const mm_msgs::DynamicObstacleMsg::ConstPtr&);
  void movingObstacles2_cb(const mm_msgs::DynamicObstacleMsg::ConstPtr&);
  void staticSpheres_cb(const mm_msgs::StaticSphereMsg::ConstPtr&);
  void resetDumpNumber_cb(const std_msgs::Bool::ConstPtr&);
  // Setting Getting Parameters
  void getMotionParameters(std::string);
  void initializeStaticSpheres();
  void initializeMovingObstacles();
  void setConstantParameters();
  void setChangingParameters();
  void setGoalParameters();
  void setForcesParams();
  void clearVariables();
  void resetInitialGuess();
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
  std::string getCurrentTimeString();
  void dumpProblem();
  // Running 
  void runNode();
};

#endif /* MPCSPHEREPLANNER_H */
