#include "mpc_action_server.h"

MPCAction::MPCAction(std::string name) :
  as_(nh_, name, boost::bind(&MPCAction::executeCB, this, _1), false),
  action_name_(name),
  mpcInterface_(name),
  // must be set according to timeStep in mpc_interface
  rate_(2.0)
{
  as_.start();
  setCollisionAvoidance();
}

MPCAction::~MPCAction()
{
}

void MPCAction::parseProblem(const mobile_mpc::mpcGoalConstPtr &goal)
{
  int i = 0;
  for(std::vector<double>::const_iterator it = goal->goal.data.begin(); it != goal->goal.data.end(); ++it)
  {
    goal_[i] = *it;
    i++;
  }
  i = 0;
  for(std::vector<double>::const_iterator it = goal->errorWeights.data.begin(); it != goal->errorWeights.data.end(); ++it)
  {
    errorWeights_[i] = *it;
    i++;
  }
  i = 0;
  for(std::vector<double>::const_iterator it = goal->mpcWeights.data.begin(); it != goal->mpcWeights.data.end(); ++it)
  {
    weights_[i] = *it;
    i++;
  }
  maxError_ = goal->maxError.data;
  safetyMargin_ = goal->safetyMargin.data;

  mpcInterface_.parseProblem(goal_, weights_, errorWeights_, safetyMargin_);
}

void MPCAction::setCollisionAvoidance()
{
    /*
    obstacleArray oa = obstacleArray({1, -3, 2.1, 1});
    mpcInterface_.setObstacles(oa);
    infPlaneArray ipa = infPlaneArray({0, -1, 0,-2, 
                                     0, 0, -1,-2, 
                                     0, 0, -1,-2}); 
    mpcInterface_.setInfPlanes(ipa);
    obstacleArray oa = obstacleArray({2.0, 1.0, 0.0, 1.0,
                                      6.0, 1.5, 0.0, 0.5, 
                                      4.0, 4.0, 2.2, 1.0, 
                                      1.5, 6.0, 0.5, 1.0, 
                                      0.5, 3.0, 0.0, 0.5
                                      });
    mpcInterface_.setObstacles(20, oa);
    obstacleArray oa = obstacleArray({
                                      2.0, 1.0, 0.0, 1.0,
                                      6.0, 1.5, 0.0, 0.5,
                                      4.0, 4.0, 2.2, 1.0,
                                      1.0, 8.0, 0.5, 1.0,
                                      7.0, 3.0, 1.5, 0.2,
                                      3.0, 5.0, 0.5, 0.5,
                                      3.5, 5.0, 0.5, 0.5,
                                      4.0, 5.0, 0.5, 0.5,
                                      4.5, 5.0, 0.5, 0.5,
                                      5.0, 5.0, 0.5, 0.5,
                                      8.0, 8.0, 0.0, 1.0,
                                      5.5, 1.5, 0.0, 0.5,
                                      0.5, 4.9, 1.2, 1.0,
                                      2.5, 2.0, 0.5, 1.0,
                                      7.5, 5.0, 0.0, 0.2,
                                      8.0, 1.0, 0.0, 1.0,
                                      5.0, 7.0, 1.7, 0.5,
                                      8.0, 4.0, 2.2, 1.0,
                                      3.5, 9.0, 0.5, 1.0,
                                      6.5, 0.3, 0.0, 0.2
                                      });
    mpcInterface_.setObstacles(80, oa);
    planeArray pa = planeArray({-2.5,   -2, 0, 2.5,   -2, 0, -2.5,   -2,   2, 
                                -2.5,   -3, 0, 2.5,   -3, 0, -2.5,   -3,   2, 
                                   4,   -2, 0,   4,   -7, 0,  4.0, -2.0, 2.0, 
                                   5,   -2, 0,   5,   -7, 0,  5.0, -2.0, 2.0, 
                                  -2, -5.5, 0,   0, -5.5, 0,   -2, -5.5, 0.7, 
                                  -2, -7.5, 0,   0, -7.5, 0,   -2, -7.5, 0.7, 
                                  -2, -5.5, 0,  -2, -7.5, 0,   -2, -5.5, 0.7, 
                                   0, -5.5, 0,   0, -7.5, 0,    0, -5.5, 0.7});
    mpcInterface_.setPlanes(pa);
    */
}
  

void MPCAction::executeCB(const mobile_mpc::mpcGoalConstPtr &goal)
{
  // helper variables
  ros::Rate r(2);
  bool success = true;
  feedback_.errorFlag = 0;

  // parse the goal message
  parseProblem(goal);
  std_msgs::Float64MultiArray t = goal->goal;
  //t = goal->errorWeights;

  // publish info to the console for the user
  ROS_INFO("Executing Forces MPC Action");

  ros::Time begin = ros::Time::now();
  while (mpcInterface_.computeError() > maxError_)
  {
    ROS_INFO("Cur error %1.5f\n", mpcInterface_.computeError());
    curUArray u_opt = mpcInterface_.solve();
    rate_.sleep();
    mpcInterface_.publishVelocities(u_opt);
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      break;
    }
  }
  ros::Duration executionTime = ros::Time::now() - begin;
  ROS_INFO("Finishing Execution");
  mpcInterface_.publishZeroVelocities();
  result_.executionTime = std_msgs::Duration();
  result_.executionTime.data = executionTime;
  result_.finalState = std_msgs::Float64MultiArray();
  if(success)
  {
    result_.success = success;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
  else
  {
    result_.success = false;
    ROS_INFO("%s : Failed", action_name_.c_str());
    as_.setSucceeded(result_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_server");

  MPCAction mpcServer("mpc_server");
  ros::spin();

  return 0;
}
