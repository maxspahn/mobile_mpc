#include "decomp.h"

Decomp::Decomp() {
  sCloud_ = nh_.subscribe("/depth_camera/depth/points", 100, &Decomp::cloud_callback, this);
  poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
  poly_pub_ = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  obs_.resize(307200);
  tfListenerPtr_ = new tf::TransformListener();
}

void Decomp::cloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2_transformed;
  sensor_msgs::PointCloud cloud_transformed;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud);
  tfListenerPtr_->waitForTransform("/odom", "/depth_camera", ros::Time::now(), ros::Duration(3.0));
  tfListenerPtr_->transformPointCloud("odom", cloud, cloud_transformed);
  cloud_to_vec(cloud_transformed);
  decompose();
}

void Decomp::cloud_to_vec(sensor_msgs::PointCloud2ConstPtr const& cloud) {
  sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  unsigned int i;
  for (i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,  ++i) {
    if (std::isnan(iter_x[0]) || std::isnan(iter_y[0]) || std::isnan(iter_z[0])) continue;
    obs_[i](0) = iter_x[0];
    obs_[i](1) = iter_y[0];
    obs_[i](2) = iter_z[0];
  }
}

void Decomp::cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  obs_.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    obs_[i](0) = cloud.points[i].x;
    obs_[i](1) = cloud.points[i].y;
    obs_[i](2) = cloud.points[i].z;
  }
}

void Decomp::decompose() {
  //std::cout << "Decompose" << std::endl;
  Vec3f seed_center = Vec3f(0, 0, 0);
  Vec3f wayPoint1 = Vec3f(3, 0, 0.5);
  Vec3f wayPoint2 = Vec3f(3, -4, 0.5);
  vec_Vec3f path;
  path.push_back(wayPoint1);
  path.push_back(wayPoint2);
  EllipsoidDecomp3D decomp_util;
  decomp_util.set_obs(obs_);
  decomp_util.set_local_bbox(Vec3f(1, 2, 1));
  decomp_util.dilate(path);

  // Visualization
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
  poly_msg.header.frame_id = "odom";
  es_msg.header.frame_id = "odom";
  poly_pub_.publish(poly_msg);
  es_pub_.publish(es_msg);

  // Inequalities
  /*
  std::cout << "Inequalities" << std::endl;
  vec_LinearConstraint3f cs = decomp_util.get_constraints();
  for(int i = 0; i < cs.size(); i++) {
    MatD3f A = cs[i].first;
    VecDf b = cs[i].second;

    printf("i: %d\n", i);
    std::cout << "start: " << (A*path[i]-b).transpose() << std::endl;
    std::cout << "end: " << (A*path[i+1]-b).transpose() << std::endl;
  }
  */
}

void Decomp::runNode() {
  ros::spin();
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  Decomp d = Decomp();
  d.runNode();
/*

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

  std::string file_name, topic_name, path_file;

  nh.param("path_file", path_file, std::string("path.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  //Read the point cloud from bag
  sensor_msgs::PointCloud cloud = read_bag<sensor_msgs::PointCloud>(file_name, topic_name);
  cloud.header.frame_id = "map";
  cloud_pub.publish(cloud);

  vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

  //Read path from txt
  vec_Vec3f path;
  if(!read_path<3>(path_file, path))
    ROS_ERROR("Fail to read a path!");

  nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
  path_msg.header.frame_id = "map";
  path_pub.publish(path_msg);

  //Using ellipsoid decomposition
  EllipsoidDecomp3D decomp_util;
  decomp_util.set_obs(obs);
  decomp_util.set_local_bbox(Vec3f(1, 2, 1));
  decomp_util.dilate(path); //Set max iteration number of 10, do fix the path

  //Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
  es_msg.header.frame_id = "map";
  es_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);

    //Convert to inequality constraints Ax < b
  auto polys = decomp_util.get_polyhedrons();
  for(size_t i = 0; i < path.size() - 1; i++) {
    const auto pt_inside = (path[i] + path[i+1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
    printf("i: %zu\n", i);
    std::cout << "A: " << cs.A() << std::endl;
    std::cout << "b: " << cs.b() << std::endl;
    std::cout << "point: " << path[i].transpose();
    if(cs.inside(path[i]))
      std::cout << " is inside!" << std::endl;
    else
      std::cout << " is outside!" << std::endl;

    std::cout << "point: " << path[i+1].transpose();
    if(cs.inside(path[i+1]))
      std::cout << " is inside!" << std::endl;
    else
      std::cout << " is outside!" << std::endl;
  }


  ros::spin();
*/

  return 0;
}
