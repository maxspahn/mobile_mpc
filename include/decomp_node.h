#ifndef DECOMP_NODE_H
#define DECOMP_NODE_H

//#include "bag_reader.hpp"
//#include "txt_reader.hpp"
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_msgs/Polyhedron.h>
#include <mm_msgs/LinearConstraint3DArray.h>
#include <decomp_ros_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <laser_assembler/AssembleScans.h>
//#include <laser_geometry/laser_geometry.h>

#include "tf/transform_listener.h"

class Decomp
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sCloud_;
  //ros::Subscriber sLaserScan_;
  ros::Publisher poly_pub_;
  ros::Publisher es_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher constraint_pub_;
  ros::Publisher laserCloud_pub_;
  ros::Publisher laserCloudT_pub_;
  ros::ServiceClient assembler_client_;
  tf::TransformListener *tfListenerPtr_;
  //laser_geometry::LaserProjection projector_;
  ros::Rate r_;
  sensor_msgs::PointCloud cameraCloud_;
  sensor_msgs::PointCloud laserCloud_;
  vec_Vec3f cameraObs_;
  vec_Vec3f laserObs_;

public:
  Decomp();
  void cloud_callback(sensor_msgs::PointCloud2ConstPtr const&);
  void runNode();
  void cloud_to_vec(const sensor_msgs::PointCloud &cloud);
  void decompose();
  void processLaserCloud();
  vec_Vec3f join_obs();
  Vec3f get_base_pos();
  mm_msgs::LinearConstraint3DArray linear_constraint_to_ros(LinearConstraint3D);
};


#endif /* DECOMP_NODE_H */
