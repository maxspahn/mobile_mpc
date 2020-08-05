#include "mpc_action_server.h"

MPCAction::MPCAction(std::string name) :
  as_(nh_, name, boost::bind(&MPCAction::executeCB, this, _1), false),
  action_name_(name),
  mpcInterface_(name),
  rate_(4)
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
    obstacleArray oa = obstacleArray({-1.5, -6, 0, 1, 
                                      -1.5, -6, 0, 1,
                                      -1.5, -6, 0, 1,
                                      -1.5, -6, 0, 1
                                      });
    planeArray pa = planeArray({-2.5,   -2, 0, 2.5,   -2, 0, -2.5,   -2,   2, 
                                -2.5,   -3, 0, 2.5,   -3, 0, -2.5,   -3,   2, 
                                   4,   -2, 0,   4,   -7, 0,  4.0, -2.0, 2.0, 
                                   5,   -2, 0,   5,   -7, 0,  5.0, -2.0, 2.0, 
                                  -2, -5.5, 0,   0, -5.5, 0,   -2, -5.5, 0.7, 
                                  -2, -7.5, 0,   0, -7.5, 0,   -2, -7.5, 0.7, 
                                  -2, -5.5, 0,  -2, -7.5, 0,   -2, -5.5, 0.7, 
                                   0, -5.5, 0,   0, -7.5, 0,    0, -5.5, 0.7});
    mpcInterface_.setObstacles(oa);
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
  ROS_INFO("Executing Acado MPC Action");

  while (mpcInterface_.computeError() > maxError_)
  {
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
  ROS_INFO("Finishing Execution");
  mpcInterface_.publishZeroVelocities();
  //result_.finalState = Float64MultiArray();
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
