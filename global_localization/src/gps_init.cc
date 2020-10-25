//
// Created by alter on 2/23/19.
//
/**
 * gps odometry时间同步？状态发布？
 */
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

#include "global_localization/gps_init.h"

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518    ///< WGS84 MINOR AXIS

enum STATUS {
  LOST_WITHOUT_GPS,
  LOST_WITH_GPS,
  NORMAL,
  WEAK
};

static inline double deg2rad(const double &deg) {
  return deg * DEG_TO_RAD;
};

static inline double rad2deg(const double &rad) {
  return rad / DEG_TO_RAD;
}

GpsInit::GpsInit() {
  gps_pos_time_ = ros::Time(0);
  gps_pos_.setZero();
  gps_heading_time_ = ros::Time(0);
  gps_heading_.setIdentity();
  has_pos_ = false;
  has_heading_ = false;
  curr_odom_pos_ << 0, 0, 0;
  for (auto &i:gps_covariance_) {
    i = 0;
  }
  status_ = STATUS::NORMAL;
}

GpsInit::GpsInit(std::string origin)
    : GpsInit() {
  std::ifstream fin(origin);
  std::string tag;
  fin >> tag;
  fin >> lla_origin_[0] >> lla_origin_[1] >> lla_origin_[2];
}

bool GpsInit::LoadMap(const std::string map_file) {
  globalmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file, *globalmap) == -1) {
    std::cout << "Couldn't read map file\n" << std::endl;
    return false;
  }
  // remove NAN
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*globalmap, *globalmap, indices);
  ROS_INFO("[init_location] Load global map");
  globalmap = utils::Downsample(globalmap, 0.5);

  // 这里发布全局地图
  sensor_msgs::PointCloud2 cloud2_msgs;
  pcl::toROSMsg(*globalmap, cloud2_msgs);
  cloud2_msgs.header.frame_id = "map";
  map_pub_.publish(cloud2_msgs);

  if (globalmap->is_dense) {
    ROS_INFO("Published global map");
    utils::PublishTransformedCloud(ros::Time::now(), map_pub_, "map", Eigen::Matrix4d::Identity(), *globalmap);
    return true;
  } else {
    PCL_ERROR("Invalid map file\n");
    return false;
  }
}

void GpsInit::RosSetup(ros::NodeHandle &nh, ros::NodeHandle &nh_) {
  // subscriber
  sub_gps_pos_ = nh.subscribe(
      "/gps/pos", 1, &GpsInit::GpsPosHandler, this);
  sub_gps_heading_ = nh.subscribe(
      "/gps/heading", 1, &GpsInit::GpsHeadingHandler, this);
  sub_odom_ = nh.subscribe(
      "/current_odom", 1, &GpsInit::CurOdomHandler, this); //  这个current_odom哪里来的？接收到之后分别按Navifix和heading发布出去
  sub_manual_ = nh.subscribe(
      "/initialpose", 1, &GpsInit::ManualHandler, this);
  sub_lost_ = nh.subscribe(
      "/localization/lost", 1, &GpsInit::LostHandler, this);
  sub_init_ = nh.subscribe(
      "/init_location/pose", 1, &GpsInit::InitHandler, this);

  //  publisher
  pub_gps2odom_ = nh.advertise<nav_msgs::Odometry>("/gps2odom", 1);
  pub_odom2gps_pos_ = nh.advertise<sensor_msgs::NavSatFix>("/odom2gps/pos", 1);
  pub_odom2gps_heading_ = nh.advertise<std_msgs::Float32>("odom2gps/heading", 1);
  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/current_map", 1); //  only when we load local map

//  pub_lost_ = nh.advertise<std_msgs::Bool>(
//      "/localization/lost", 1
//  );
}

void GpsInit::Process() {
  if (has_pos_) {
    auto ros_time_diff = gps_pos_time_ - gps_heading_time_;
    double time_diff = ros_time_diff.toSec();

    Eigen::Matrix4d mx = Eigen::Matrix4d::Identity();
    mx.block<3, 1>(0, 3) = Eigen::Vector3d(gps_pos_(0), gps_pos_(1), gps_pos_(2));
/*    if (has_heading_ && std::fabs(time_diff) < 0.5) {
      std::cout << "heading:" << gps_heading_.matrix() << std::endl;
      mx.block<3, 3>(0, 0) = gps_heading_.toRotationMatrix();
      utils::PublishOdometry(gps_heading_time_, pub_gps2odom_, "map", "gps", mx.matrix());
//      quaternion.x = heading_quaternion_.x;
//      quaternion.y = heading_quaternion_.y;
//      quaternion.z = heading_quaternion_.w;
//      quaternion.w = heading_quaternion_.z;

//      quaternion.x = gps_heading_.x();
//      quaternion.y = gps_heading_.y();
//      quaternion.z = gps_heading_.z();
//      quaternion.w = gps_heading_.w();

//      double input_roll, input_pitch, input_yaw;
//      tf::Quaternion input_orientation;
//      tf::quaternionMsgToTF(heading_quaternion_, input_orientation);
//      tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);
//
//      //input_yaw -= M_PI / 2;
//      std::cout << "input yaw: " << input_yaw << std::endl;
//      quaternion = tf::createQuaternionMsgFromYaw(input_yaw);
    } else*/ if (_orientation_ready) {
      std::cout << "_quat:" << _quat.w << ", " << _quat.z << std::endl;

      gnss_matrix.block<3, 3>(0, 0) = Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z).toRotationMatrix();
      utils::PublishOdometry(gps_heading_time_, pub_gps2odom_, "map", "gps", mx.matrix());
    }

/*    nav_msgs::Odometry m;
    m.header.stamp = gps_pos_time_;
    m.header.frame_id = "map";
    m.pose.covariance = gps_covariance_;
    m.pose.pose.position.x = gps_pos_(0);
    m.pose.pose.position.y = gps_pos_(1);
    m.pose.pose.position.z = gps_pos_(2);
    m.pose.pose.orientation = quaternion;

    //tf::poseEigenToMsg(e, m.pose.pose);
    pub_gps2odom_.publish(m);*/


    if (status_ == STATUS::LOST_WITHOUT_GPS) {
      status_ = STATUS::LOST_WITH_GPS;
    }

    has_pos_ = false;
    has_heading_ = false;
    _orientation_ready = false;
  }
}

void GpsInit::GpsPosHandler(const sensor_msgs::NavSatFix::ConstPtr &gps_pos) {
  if (std::isnan(gps_pos->latitude + gps_pos->longitude + gps_pos->altitude)) {
    return;
  }
  gps_covariance_[5] = gps_pos->status.status;
  gps_pos_time_ = gps_pos->header.stamp;
  gps_covariance_[0] = gps_pos->position_covariance[0];
  gps_covariance_[1] = gps_pos->position_covariance[1];
  gps_covariance_[2] = gps_pos->position_covariance[2];
  gps_covariance_[6] = gps_pos->position_covariance[3];
  gps_covariance_[7] = gps_pos->position_covariance[4];
  gps_covariance_[8] = gps_pos->position_covariance[5];
  gps_covariance_[12] = gps_pos->position_covariance[6];
  gps_covariance_[13] = gps_pos->position_covariance[7];
  gps_covariance_[14] = gps_pos->position_covariance[8];

  Eigen::Vector3d lla = GpsMsg2Eigen(*gps_pos);
  Eigen::Vector3d ecef = LLA2ECEF(lla);
  Eigen::Vector3d enu = ECEF2ENU(ecef);
  gps_pos_ = enu;
  has_pos_ = true;

  //std::cout << gps_pos_.transpose() << std::endl;
  //  根据运动计算GPS YAW
  double distance_diff = sqrt(pow(enu(1) - enu(1), 2) +
      pow(enu(0) - prev_pos_(0), 2));
  if (distance_diff > 0.1) {
    yaw = atan2(enu(1) - prev_pos_(1),
                enu(0) - prev_pos_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    prev_pos_ = enu;
    _orientation_ready = true;
  }

  bool good_status = gps_pos->status.status == 4;
  if (curr_odom_pos_.isZero())
    return;
  Eigen::Vector3d position_diff = curr_odom_pos_ - gps_pos_;
  position_diff.z() = 0;
  double distance = position_diff.norm();
  std::cout << gps_pos->header.stamp << ": " << gps_pos_.transpose() << " "
            << distance << " " << int(gps_pos->status.status);
  for (int i = 0; i < 8; ++i) {
    std::cout << " " << gps_pos->position_covariance[i];
  }
  std::cout << std::endl;


//  Eigen::Matrix4d mx = Eigen::Matrix4d::Identity();
//  mx.block<3, 3>(0, 0) =
//      Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z).toRotationMatrix();
//  mx.block<3, 1>(0, 3) = Eigen::Vector3d(gps_pos_(0), gps_pos_(1), gps_pos_(2)).matrix();
//  utils::PublishOdometry(gps_heading_time_, pub_gps2odom_, "map", "gps", mx.matrix());

//  if ((good_status && distance > 50) || (!good_status && distance > 100)) {
//    std_msgs::Bool lost_msgs;
//    lost_msgs.data = true;
//    std::cout << "[" << gps_pos->header.stamp << "] inconsistent with gps "
//              << distance
//              << std::endl;
//    ros::Rate(1).sleep();
//    pub_lost_.publish((lost_msgs));
//  }
}

void GpsInit::GpsHeadingHandler(const geometry_msgs::QuaternionStamped::ConstPtr &gps_heading) {
  auto q = gps_heading->quaternion;
  if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
    ROS_ERROR("Nan heading...");
    return;
  }
  gps_heading_time_ = gps_heading->header.stamp;
  gps_heading_ = Eigen::Quaterniond(q.z, q.x, q.y, q.w); // gps heading to map
  has_heading_ = true;
}

void GpsInit::ManualHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_estimate_msgs) {
  for (int i = 0; i < 5; i++) {
    ros::Rate(1).sleep();
  }
}

void GpsInit::CurOdomHandler(const nav_msgs::Odometry::ConstPtr &odom) {
  {
    sensor_msgs::NavSatFix msgs;
    msgs.header.stamp = odom->header.stamp;
    msgs.header.frame_id = "gps";

    tf::pointMsgToEigen(odom->pose.pose.position, curr_odom_pos_);

    Eigen::Vector3d ecef = ENU2ECEF(curr_odom_pos_);
    Eigen::Vector3d lla = ECEF2LLA(ecef);

    msgs.latitude = lla[0];
    msgs.longitude = lla[1];
    msgs.altitude = lla[2];
    msgs.status.status = status_;

    pub_odom2gps_pos_.publish(msgs);
  }
  {
    std_msgs::Float32 msgs;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(odom->pose.pose.orientation, q);
    Eigen::Vector3d unit(1, 0, 0);
    unit = q.toRotationMatrix() * unit;
    double angle = atan2(unit.y(), unit.x());
    msgs.data = 90 - rad2deg(angle);
    if (msgs.data < 0) msgs.data += 360;
    pub_odom2gps_heading_.publish(msgs);
  }
}

void GpsInit::LostHandler(const std_msgs::Bool::ConstPtr &lost) {
  if (status_ == STATUS::NORMAL || status_ == STATUS::WEAK)
    status_ = STATUS::LOST_WITHOUT_GPS;
}

void GpsInit::InitHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init) {
  ros::Rate(0.5).sleep();
  ros::spinOnce(); //clear message cache
  status_ = STATUS::NORMAL;
}

Eigen::Vector3d GpsInit::GpsMsg2Eigen(const sensor_msgs::NavSatFix &gps_msgs) {
  Eigen::Vector3d
      lla(gps_msgs.latitude, gps_msgs.longitude, gps_msgs.altitude);
  return lla;
}

//https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
Eigen::Vector3d GpsInit::LLA2ECEF(const Eigen::Vector3d &lla) {
  Eigen::Vector3d ecef;
  double lat = deg2rad(lla.x());
  double lon = deg2rad(lla.y());
  double alt = lla.z();
  double earth_r = pow(EARTH_MAJOR, 2)
      / sqrt(pow(EARTH_MAJOR * cos(lat), 2) + pow(EARTH_MINOR * sin(lat), 2));
  ecef.x() = (earth_r + alt) * cos(lat) * cos(lon);
  ecef.y() = (earth_r + alt) * cos(lat) * sin(lon);
  ecef.z() = (pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * sin(lat);

  return ecef;
}

Eigen::Vector3d GpsInit::ECEF2LLA(const Eigen::Vector3d &ecef) {
  double e =
      sqrt((pow(EARTH_MAJOR, 2) - pow(EARTH_MINOR, 2)) / pow(EARTH_MAJOR, 2));
  double e_ =
      sqrt((pow(EARTH_MAJOR, 2) - pow(EARTH_MINOR, 2)) / pow(EARTH_MINOR, 2));
  double p = sqrt(pow(ecef.x(), 2) + pow(ecef.y(), 2));
  double theta = atan2(ecef.z() * EARTH_MAJOR, p * EARTH_MINOR);

  double lon = atan2(ecef.y(), ecef.x());
  double lat = atan2((ecef.z() + pow(e_, 2) * EARTH_MINOR * pow(sin(theta), 3)),
                     p - pow(e, 2) * EARTH_MAJOR * pow(cos(theta), 3));
  double earth_r = pow(EARTH_MAJOR, 2)
      / sqrt(pow(EARTH_MAJOR * cos(lat), 2) + pow(EARTH_MINOR * sin(lat), 2));
  double alt = p / cos(lat) - earth_r;
  Eigen::Vector3d lla(rad2deg(lat), rad2deg(lon), alt);
  return lla;
}

//https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
Eigen::Vector3d GpsInit::ECEF2ENU(const Eigen::Vector3d &ecef) {
  double lat = deg2rad(lla_origin_.x());
  double lon = deg2rad(lla_origin_.y());

  Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
  Eigen::Matrix3d r;
  r << -sin(lon), cos(lon), 0,
      -cos(lon) * sin(lat), -sin(lat) * sin(lon), cos(lat),
      cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);

  Eigen::Vector3d enu;
  enu = ecef + t;
  enu = r * enu;
  return enu;
}

Eigen::Vector3d GpsInit::ENU2ECEF(const Eigen::Vector3d &enu) {
  double lat = deg2rad(lla_origin_.x());
  double lon = deg2rad(lla_origin_.y());

  Eigen::Vector3d t = LLA2ECEF(lla_origin_);
  Eigen::Matrix3d r;
  r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
      cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
      0, cos(lat), sin(lat);
  Eigen::Vector3d ecef;
  ecef = r * enu + t;
  return ecef;
}

