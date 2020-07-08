#ifndef MPC_ACTION_SERVER_H
#define MPC_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mobile_mpc/mpcAction.h>
#include "mpc_interface.h"

#include "std_msgs/Float64MultiArray.h"

class MPCAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mobile_mpc::mpcAction> as_;
  std::string action_name_;
  mobile_mpc::mpcFeedback feedback_;
  mobile_mpc::mpcResult result_;
  MpcInterface mpcInterface_;
  goalArray goal_;
  weightArray weights_;
  errorWeightArray errorWeights_;
  double maxError_;

public:
  MPCAction(std::string);
  ~MPCAction();
  void executeCB(const mobile_mpc::mpcGoalConstPtr&);
  void parseProblem(const mobile_mpc::mpcGoalConstPtr&);
};

#endif /* MPC_ACTION_SERVER_H */
