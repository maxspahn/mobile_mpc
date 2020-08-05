#include "decomp_node.h"

Decomp::Decomp() : r_(10) {
  sCloud_ = nh_.subscribe("/octomap_point_cloud_centers", 100, &Decomp::cloud_callback, this);
  poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
  es_pub_ = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("obs_cloud", 1, true);
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_base", 1, true));
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_mid", 1, true));
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_ee", 1, true));
  tfListenerPtr_ = new tf::TransformListener();
}

void Decomp::cloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, octoCloud_);
}

void Decomp::cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  // must be optimized
  obs_.clear();
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    if (cloud.points[i].z > 0.1) {
      Vec3f point = Vec3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      obs_.push_back(point);
    }
  }
  //printf("Size points in pointcloud obs_ %d\n", obs_.size());
  sensor_msgs::PointCloud obs_msg = vec_to_cloud(obs_);
  obs_msg.header.frame_id = std::string("odom");
  cloud_pub_.publish(obs_msg);
  /*
  for (unsigned int i = 0; i < obs_.size(); ++i) {
    printf("Obstacle[%d] : %1.2f, %1.2f, %1.2f\n", i, obs_[i](0), obs_[i](1), obs_[i](2));
  }
  */
}

sensor_msgs::PointCloud Decomp::vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());
  for (int i = 0; i < pts.size(); ++i) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

Vec3f Decomp::get_link_pos(std::string linkName) {
  Vec3f pos = Vec3f(0, 0, 0);
  tf::StampedTransform strans;
  tfListenerPtr_->lookupTransform("odom", linkName, ros::Time(0), strans);
  tf::Vector3 posRos = strans.getOrigin();
  pos[0] = posRos[0];
  pos[1] = posRos[1];
  pos[2] = posRos[2];
  return pos;
}


void Decomp::decompose() {
  tfListenerPtr_->waitForTransform("/odom", "/depth_camera", ros::Time::now(), ros::Duration(1.0));
  sensor_msgs::PointCloud cloud_transformed;
  if (octoCloud_.points.size() == 0) return;
  tfListenerPtr_->transformPointCloud("odom", octoCloud_, cloud_transformed);
  cloud_to_vec(cloud_transformed);
  Vec3f seed_center = get_link_pos("base_link");
  vec_Vec3f seed_centers;
  seed_centers.push_back(get_link_pos("top_mount_bottom"));
  seed_centers.push_back(get_link_pos("mmrobot_link4"));
  seed_centers.push_back(get_link_pos("mmrobot_link6"));
  /*
  // Using path
  Vec3f wayPoint1 = get_link_pos("top_mount_bottom");
  Vec3f wayPoint2 = get_link_pos("mmrobot_link0");
  Vec3f wayPoint3 = get_link_pos("mmrobot_link4");
  Vec3f wayPoint4 = get_link_pos("mmrobot_link6");
  Vec3f wayPoint5 = get_link_pos("mmrobot_virtual_hand");
  vec_Vec3f path;
  path.push_back(wayPoint1);
  path.push_back(wayPoint2);
  path.push_back(wayPoint3);
  path.push_back(wayPoint4);
  path.push_back(wayPoint5);
  EllipsoidDecomp3D decomp_util;
  decomp_util.set_obs(obs_);
  decomp_util.set_local_bbox(Vec3f(1.5, 1.5, 1.5));
  decomp_util.dilate(path);
  vec_E<Ellipsoid3D> es = decomp_util.get_ellipsoids();
  vec_E<Polyhedron3D> polys = decomp_util.get_polyhedrons();
  */
  

  // Using seeds
  vec_E<Ellipsoid3D> es;
  vec_E<Polyhedron3D> polys;
  std::array<double, 3> sphere_sizes = {5.0, 1.0, 1.0};
  for (size_t i = 0; i < seed_centers.size(); ++i) {
    SeedDecomp3D decomp_util(seed_centers[i]);
    decomp_util.set_obs(obs_);
    //decomp_util.set_local_bbox(Vec3f(3.5, 3.5, 3.5));
    decomp_util.dilate(sphere_sizes[i]);
    es.push_back(decomp_util.get_ellipsoid());
    polys.push_back(decomp_util.get_polyhedron());
  }

  // Visualization
  //cloud_transformed.header.frame_id = "odom";
  //cloud_pub_.publish(cloud_transformed);
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
  for(size_t i = 0; i < polys.size(); i++) {
    //const auto pt_inside = (path[i] + path[i+1]) / 2;
    LinearConstraint3D cs(seed_center, polys[i].hyperplanes());
    mm_msgs::LinearConstraint3DArray constraints = linear_constraint_to_ros(cs);
    constraint_pub_[i].publish(constraints);
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
