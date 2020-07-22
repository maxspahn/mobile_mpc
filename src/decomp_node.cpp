#include "decomp_node.h"

Decomp::Decomp() : r_(10) {
  sCloud_ = nh_.subscribe("/depth_camera/depth/points", 100, &Decomp::cloud_callback, this);
  poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
  es_pub_ = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  constraint_pub_ = nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints", 1, true);
  obs_.resize(307200);
  tfListenerPtr_ = new tf::TransformListener();
}

void Decomp::cloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud_);
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
  tfListenerPtr_->waitForTransform("/odom", "/depth_camera", ros::Time::now(), ros::Duration(1.0));
  sensor_msgs::PointCloud cloud_transformed;
  if (cloud_.points.size() == 0) return;
  tfListenerPtr_->transformPointCloud("odom", cloud_, cloud_transformed);
  cloud_to_vec(cloud_transformed);
   
  Vec3f seed_center = Vec3f(0, -4, 0.5);
  Vec3f wayPoint1 = Vec3f(3, 0, 0.5);
  Vec3f wayPoint2 = Vec3f(3, -4, 0.5);
  Vec3f wayPoint3 = Vec3f(-3, -4.4, 0.5);
  Vec3f wayPoint4 = Vec3f(-3, -7.4, -0.8);
  vec_Vec3f path;
  path.push_back(wayPoint1);
  path.push_back(wayPoint2);
  path.push_back(wayPoint3);
  path.push_back(wayPoint4);
  //EllipsoidDecomp3D decomp_util;
  SeedDecomp3D decomp_util(seed_center);
  decomp_util.set_obs(obs_);
  decomp_util.set_local_bbox(Vec3f(5, 5, 2));
  //decomp_util.dilate(path);
  decomp_util.dilate(5.0);


  vec_E<Ellipsoid3D> es;
  vec_E<Polyhedron3D> polys;
  es.push_back(decomp_util.get_ellipsoid());
  polys.push_back(decomp_util.get_polyhedron());

  // Visualization
  cloud_transformed.header.frame_id = "odom";
  cloud_pub_.publish(cloud_transformed);
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(es);
  poly_msg.header.frame_id = "odom";
  es_msg.header.frame_id = "odom";
  poly_pub_.publish(poly_msg);
  es_pub_.publish(es_msg);

  // Inequalities
    //Convert to inequality constraints Ax < b
  //auto polys = decomp_util.get_polyhedrons();
  //for(size_t i = 0; i < path.size() - 1; i++) {
  int nbPolys = 1;
  for(size_t i = 0; i < nbPolys; i++) {
    //const auto pt_inside = (path[i] + path[i+1]) / 2;
    LinearConstraint3D cs(seed_center, polys[i].hyperplanes());
    mm_msgs::LinearConstraint3DArray constraints = linear_constraint_to_ros(cs);
    constraint_pub_.publish(constraints);
/*
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
*/
  }
}

mm_msgs::LinearConstraint3DArray Decomp::linear_constraint_to_ros(LinearConstraint3D cs)
{
  mm_msgs::LinearConstraint3DArray constraints;
  for (int i = 0; i < cs.A().rows() ; ++i) {
    mm_msgs::LinearConstraint3D cm;
    cm.A[0] = cs.A()(i, 0);
    cm.A[1] = cs.A()(i, 1);
    cm.A[2] = cs.A()(i, 2);
    cm.b = cs.b()(i);
    constraints.constraints.push_back(cm);
  }
  return constraints;
}


void Decomp::runNode() {
  while (ros::ok())
  {
    ros::spinOnce();
    decompose();
    r_.sleep();
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  Decomp d = Decomp();
  d.runNode();
  return 0;
}
