#include <ros/ros.h>
#include <mpc_acado.h>

int main(int argc, char **argv)
{
  try
  {
    std::string name = "mpc_acado";
    std::array<double, 10> goal = {-2.0, 2.0, -1.0, 0.5, -0.75, 0.5, -1.0, 0.0, 2.1, 0.75};
    //std::array<double, 10> goal = {2.0, -3.0, 1.0, 0.5, 0.75, 0.5, -1.5, 0.0, 2.6, 0.75};
    ros::init(argc, argv, ros::this_node::getName());
    MpcAcado myMpc(name);
    myMpc.runNode(goal);
//    ros::spin();
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("predictive_control_node: Error occured: %s ", e.what());
    exit(1);
  }

  return 0;
}
