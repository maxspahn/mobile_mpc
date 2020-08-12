#ifndef DECOMP_H
#define DECOMP_H

//#include "bag_reader.hpp"
//#include "txt_reader.hpp"
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_msgs/Polyhedron.h>
#include <decomp_ros_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

#include "tf/transform_listener.h"

class Decomp
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sCloud_;
  ros::Publisher poly_pub_;
  ros::Publisher es_pub_;
  ros::Publisher cloud_pub_;
  tf::TransformListener *tfListenerPtr_;
  ros::Rate r_;
  sensor_msgs::PointCloud cloud_;
  vec_Vec3f obs_;

public:
  Decomp();
  void cloud_callback(sensor_msgs::PointCloud2ConstPtr const& cloud_msg);
  void runNode();
  void cloud_to_vec(sensor_msgs::PointCloud2ConstPtr const& cloud);
  void cloud_to_vec(const sensor_msgs::PointCloud &cloud);
  void decompose();
};

#endif /* DECOMP_H */
