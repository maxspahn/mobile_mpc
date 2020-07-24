#include "decomp_node.h"

Decomp::Decomp() : r_(10) {
  ros::service::waitForService("assemble_scans");
  sCloud_ = nh_.subscribe("/depth_camera/depth/points", 100, &Decomp::cloud_callback, this);
  laserCloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("laserCloud", 1, true);
  laserCloudT_pub_ = nh_.advertise<sensor_msgs::PointCloud>("laserCloudT", 1, true);
  poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
  es_pub_ = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  constraint_pub_ = nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints", 1, true);
  assembler_client_ = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  tfListenerPtr_ = new tf::TransformListener();
}

void Decomp::cloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cameraCloud_);
}

void Decomp::cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  cameraObs_.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    cameraObs_[i](0) = cloud.points[i].x;
    cameraObs_[i](1) = cloud.points[i].y;
    cameraObs_[i](2) = cloud.points[i].z;
  }
}

void Decomp::processLaserCloud() {
  laser_assembler::AssembleScans srv;
  srv.request.begin = ros::Time(0.0);
  srv.request.end   = ros::Time::now();
  if (assembler_client_.call(srv)) {
    tfListenerPtr_->waitForTransform("/odom", "/front_laser", ros::Time::now(), ros::Duration(1.0));
    sensor_msgs::PointCloud cloud = srv.response.cloud;
    sensor_msgs::PointCloud cloud_transformed;
    cloud_transformed.header.frame_id = "front_laser";
    laserCloud_pub_.publish(cloud);
    tfListenerPtr_->transformPointCloud("odom", cloud, cloud_transformed);
    laserObs_.resize(cloud_transformed.points.size());
    cloud_transformed.header.frame_id = "odom";
    laserCloudT_pub_.publish(cloud_transformed);
    for (unsigned int i = 0; i < cloud_transformed.points.size(); ++i) {
      laserObs_[i](0) = cloud_transformed.points[i].x;
      laserObs_[i](1) = cloud_transformed.points[i].y;
      laserObs_[i](2) = cloud_transformed.points[i].z;
    }
  }
}

vec_Vec3f Decomp::join_obs() {
  vec_Vec3f obs;
  obs.resize(cameraObs_.size() + laserObs_.size());
  for (unsigned int i = 0; i < cameraObs_.size(); ++i) {
    obs[i](0) = cameraObs_[i](0);
    obs[i](1) = cameraObs_[i](1);
    obs[i](2) = cameraObs_[i](2);
  }
  for (unsigned int i = 0; i < laserObs_.size(); ++i) {
    obs[i + cameraObs_.size()](0) = laserObs_[i](0);
    obs[i + cameraObs_.size()](1) = laserObs_[i](1);
    obs[i + cameraObs_.size()](2) = laserObs_[i](2);
  }
  return obs;
}

Vec3f Decomp::get_base_pos() {
  Vec3f pos_base = Vec3f(0, 0, 0);
  tf::StampedTransform strans;
  tfListenerPtr_->lookupTransform("odom", "top_mount_bottom", ros::Time(0), strans);
  tf::Vector3 posBase = strans.getOrigin();
  pos_base[0] = posBase[0];
  pos_base[1] = posBase[1];
  pos_base[2] = posBase[2];
  return pos_base;
}



void Decomp::decompose() {
  tfListenerPtr_->waitForTransform("/odom", "/depth_camera", ros::Time::now(), ros::Duration(1.0));
  sensor_msgs::PointCloud cloud_transformed;
  if (cameraCloud_.points.size() == 0) return;
  tfListenerPtr_->transformPointCloud("odom", cameraCloud_, cloud_transformed);
  cloud_to_vec(cloud_transformed);
  processLaserCloud();
  vec_Vec3f obs = join_obs();
   
  Vec3f seed_center = get_base_pos();
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
  decomp_util.set_obs(obs);
  decomp_util.set_local_bbox(Vec3f(5, 5, 2));
  //decomp_util.dilate(path);
  decomp_util.dilate(5.0);


  vec_E<Ellipsoid3D> es;
  vec_E<Polyhedron3D> polys;
  es.push_back(decomp_util.get_ellipsoid());
  polys.push_back(decomp_util.get_polyhedron());

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
