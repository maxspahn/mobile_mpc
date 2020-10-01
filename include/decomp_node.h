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
#include "mm_msgs/NurbsEval2D.h"
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
  ros::Subscriber sLaserCloud_;
  ros::Subscriber subGlobalPath_;
  std::vector<ros::Publisher> poly_pub_;
  std::vector<ros::Publisher> es_pub_;
  ros::Publisher cloud_pub_;
  std::vector<ros::Publisher> constraint_pub_;
  tf::TransformListener *tfListenerPtr_;
  //laser_geometry::LaserProjection projector_;
  ros::Rate r_;
  sensor_msgs::PointCloud octoCloud_;
  sensor_msgs::PointCloud laserCloud_;
  vec_Vec3f obs_;
  std::string reference_frame_;
  std::vector<std::array<double, 3>> globalPath_;

public:
  Decomp();
  void cloud_callback(sensor_msgs::PointCloud2ConstPtr const&);
  void globalPath_cb(mm_msgs::NurbsEval2D::ConstPtr const&);
  void lasercloud_callback(sensor_msgs::PointCloud2ConstPtr const&);
  void runNode();
  void cloud_to_vec(const sensor_msgs::PointCloud &);
  sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts);
  void decompose();
  void processLaserCloud();
  vec_Vec3f get_link_pos(std::string);
  mm_msgs::LinearConstraint3DArray linear_constraint_to_ros(LinearConstraint3D);
  void checkTfListener();
};


#endif /* DECOMP_NODE_H */
