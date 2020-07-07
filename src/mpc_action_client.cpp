#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mobile_mpc/mpcAction.h>
#include <std_msgs/Float64MultiArray.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_mpc_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<mobile_mpc::mpcAction> ac("mpc_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  mobile_mpc::mpcGoal goal;
  // setup the goal
  std::vector<double> targetData = {-1.0, 1.0, 0.0, 1.2, -0.3, 0.0, -1.0, 0.0, 2.1, 0.0};
  std_msgs::Float64MultiArray targetConfig;
  targetConfig.layout.dim.push_back(std_msgs::MultiArrayDimension());
  targetConfig.layout.dim[0].size = targetData.size();
  targetConfig.layout.dim[0].stride = 1;
  targetConfig.layout.dim[0].label = "x";
  targetConfig.data.clear();
  targetConfig.data.insert(targetConfig.data.end(), targetData.begin(), targetData.end());
  goal.goal = targetConfig;
  // setup the weights
  // w_x, w_o, w_q, w_u, w_qdot, w_slack
  std::vector<double> weightsData = {150.0, 10.0, 1.0, 0.5, 20.0, 0.0};
  std_msgs::Float64MultiArray weightsConfig;
  weightsConfig.layout.dim.push_back(std_msgs::MultiArrayDimension());
  weightsConfig.layout.dim[0].size = weightsData.size();
  weightsConfig.layout.dim[0].stride = 1;
  weightsConfig.layout.dim[0].label = "x";
  weightsConfig.data.clear();
  weightsConfig.data.insert(weightsConfig.data.end(), weightsData.begin(), weightsData.end());
  goal.mpcWeights = weightsConfig;
  // setup up the maxError
  double maxErrorData = 0.2;
  std_msgs::Float64 maxErrorConfig;
  maxErrorConfig.data = maxErrorData;
  goal.maxError = maxErrorConfig;
  // setup the errorWeights
  std::vector<double> errorWeightsData = {1.0, 0.5, 1.0};
  std_msgs::Float64MultiArray errorWeightsConfig;
  errorWeightsConfig.layout.dim.push_back(std_msgs::MultiArrayDimension());
  errorWeightsConfig.layout.dim[0].size = errorWeightsData.size();
  errorWeightsConfig.layout.dim[0].stride = 1;
  errorWeightsConfig.layout.dim[0].label = "x";
  errorWeightsConfig.data.clear();
  errorWeightsConfig.data.insert(errorWeightsConfig.data.end(), errorWeightsData.begin(), errorWeightsData.end());
  goal.errorWeights = errorWeightsConfig;
  
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

