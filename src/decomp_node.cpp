#include "decomp_node.h"

Decomp::Decomp() : r_(1) {
  sCloud_ = nh_.subscribe("/octomap_point_cloud_centers", 100, &Decomp::cloud_callback, this);
  sLaserCloud_ = nh_.subscribe("/laser_pointcloud", 100, &Decomp::lasercloud_callback, this);
  poly_pub_.push_back(nh_.advertise<decomp_ros_msgs::PolyhedronArray>("base1_polyhedron", 1, true));
  poly_pub_.push_back(nh_.advertise<decomp_ros_msgs::PolyhedronArray>("base2_polyhedron", 1, true));
  poly_pub_.push_back(nh_.advertise<decomp_ros_msgs::PolyhedronArray>("mid_polyhedron", 1, true));
  poly_pub_.push_back(nh_.advertise<decomp_ros_msgs::PolyhedronArray>("ee_polyhedron", 1, true));
  es_pub_.push_back(nh_.advertise<decomp_ros_msgs::EllipsoidArray>("base1_elipsoid", 1, true));
  es_pub_.push_back(nh_.advertise<decomp_ros_msgs::EllipsoidArray>("base2_elipsoid", 1, true));
  es_pub_.push_back(nh_.advertise<decomp_ros_msgs::EllipsoidArray>("mid_elipsoid", 1, true));
  es_pub_.push_back(nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ee_elipsoid", 1, true));
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("obs_cloud", 1, true);
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_base1", 1, true));
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_base2", 1, true));
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_mid", 1, true));
  constraint_pub_.push_back(nh_.advertise<mm_msgs::LinearConstraint3DArray>("constraints_ee", 1, true));
  subGlobalPath_ = nh_.subscribe("/spline/globalPath", 10, &Decomp::globalPath_cb, this);
  tfListenerPtr_ = new tf::TransformListener();
  nh_.getParam("/reference_frame", reference_frame_);
  globalPath_.resize(15);
  checkTfListener();
}

void Decomp::checkTfListener() {
  ROS_INFO("Cheking tf listener");
  unsigned int counter = 0;
  while(true) {
    try {
      tfListenerPtr_->waitForTransform(reference_frame_, "/base_link", ros::Time::now(), ros::Duration(1.0));
      tfListenerPtr_->waitForTransform(reference_frame_, "/octomap", ros::Time::now(), ros::Duration(1.0));
      break;
    }
    catch (tf2::ConnectivityException e) {
      counter++;
    }
    catch (tf2::LookupException e) {
      counter = counter + 1;
      if (counter > 50) {
        ROS_WARN("You requested to use %s as the reference frame. It is not available. Consider to start a localization. 'odom' will used instead for now", reference_frame_.c_str());
        ROS_WARN("%s", e.what());
        reference_frame_ = "odom";
      }
    }
    catch (tf2::ExtrapolationException e) {
      ROS_INFO("Wating for tf to come up...");
    }
  }
}

void Decomp::globalPath_cb(const mm_msgs::NurbsEval2D::ConstPtr& evalNurbs)
{
  for (unsigned int i = 0; i < 15; ++i) {
    globalPath_[i][0] = evalNurbs->evaluations[i].x;
    globalPath_[i][1] = evalNurbs->evaluations[i].y;
  }
}

void Decomp::cloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud_transformed;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud_transformed);
  cloud_transformed.header.stamp = ros::Time::now();
  if (cloud_transformed.points.size() == 0) return;
  int counter = 0;
  while (true) {
    try {
      counter++;
      if (counter > 10) {
	 ROS_WARN("Could tranform octomap pointcloud");
	 break;
      }
      tfListenerPtr_->transformPointCloud(reference_frame_, cloud_transformed, octoCloud_);
      break;
    }
    catch (tf2::LookupException e) {
      ROS_WARN("%s", e.what());
      continue;
    }
    catch (tf2::ExtrapolationException e) {
      ROS_WARN("%s", e.what());
      continue;
    }
  }
}

void Decomp::lasercloud_callback (sensor_msgs::PointCloud2ConstPtr const& cloud_msg){
  sensor_msgs::PointCloud cloud_transformed;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud_transformed);
  //cloud_transformed.header.stamp = ros::Time::now();
  if (cloud_transformed.points.size() == 0) return;
  int counter = 0;
  while (true) {
    try {
      counter++;
      if (counter > 10) {
	 ROS_WARN("Could tranform laser pointcloud");
	 break;
      }
      tfListenerPtr_->transformPointCloud(reference_frame_, cloud_transformed, laserCloud_);
      break;
    }
    catch (tf2::LookupException e) {
      ROS_WARN("%s", e.what());
      continue;
    }
    catch (tf2::ExtrapolationException e) {
      ROS_WARN("%s", e.what());
      continue;
    }
  }
}

void Decomp::cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  // must be optimized
  //obs_.clear();
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    if (cloud.points[i].z > 0.1) {
      Vec3f point = Vec3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      obs_.push_back(point);
    }
  }
  //printf("Size points in pointcloud obs_ %d\n", obs_.size());
  sensor_msgs::PointCloud obs_msg = vec_to_cloud(obs_);
  obs_msg.header.frame_id = reference_frame_;
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

vec_Vec3f Decomp::get_link_pos(std::string linkName) {
  vec_Vec3f pos;
  if (linkName.compare("base_sphere_blablabla") == 0) {
    Vec3f firstPos = Vec3f(globalPath_[0][0], globalPath_[0][1], 0.2);
    Vec3f secondPos = Vec3f(globalPath_[10][0], globalPath_[10][1], 0.2);
    pos.push_back(firstPos);
    pos.push_back(secondPos);
  }
  else {
    Vec3f firstPos = Vec3f(0, 0, 0);
    tf::StampedTransform strans;
    tfListenerPtr_->lookupTransform(reference_frame_, linkName, ros::Time(0.0), strans);
    tf::Vector3 posRos = strans.getOrigin();
    firstPos[0] = posRos[0];
    firstPos[1] = posRos[1];
    firstPos[2] = posRos[2];
    pos.push_back(firstPos + Vec3f(0.0, 0.0, 0.0));
    pos.push_back(firstPos + Vec3f(0.0, 0.0, -0.1));
  }
  return pos;
}


void Decomp::decompose() {
  obs_.clear();
  cloud_to_vec(octoCloud_);
  cloud_to_vec(laserCloud_);
  std::vector<vec_Vec3f> paths;
  paths.push_back(get_link_pos("base_sphere"));
  paths.push_back(get_link_pos("base_sphere"));
  paths.push_back(get_link_pos("mmrobot_link2_sphere"));
  paths.push_back(get_link_pos("mmrobot_link7_sphere"));
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
  std::array<double, 4> sphere_sizes = {2.5, 2.5, 1.0, 1.0};
  for (size_t i = 0; i < paths.size(); ++i) {
    //SeedDecomp3D decomp_util(seed_centers[i]);
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(obs_);
    decomp_util.set_local_bbox(Vec3f(sphere_sizes[i], sphere_sizes[i], sphere_sizes[i]));
    decomp_util.dilate(paths[i]);
    //es.push_back(decomp_util.get_ellipsoid());
    //polys.push_back(decomp_util.get_polyhedron());
    es = decomp_util.get_ellipsoids();
    polys = decomp_util.get_polyhedrons();
    // Visualization
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(es);
    poly_msg.header.frame_id = reference_frame_;
    es_msg.header.frame_id = reference_frame_;
    poly_pub_[i].publish(poly_msg);
    es_pub_[i].publish(es_msg);
    // Inequalities
    const auto pt_inside = (paths[i][0] + paths[i][1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[0].hyperplanes());
    mm_msgs::LinearConstraint3DArray constraints = linear_constraint_to_ros(cs);
    constraint_pub_[i].publish(constraints);
    polys.clear();
    es.clear();
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
