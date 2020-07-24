#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
using namespace laser_assembler;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");
  while (ros::ok()) {
    AssembleScans srv;
    srv.request.begin = ros::Time(0,0);
    srv.request.end   = ros::Time::now();
    if (client.call(srv))
      printf("Got cloud with %u points\n", srv.response.cloud.points.size());
    else
      printf("Service call failed\n");
    ros::spinOnce();
  }
  return 0;
}
