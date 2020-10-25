//
// Created by alter on 2/23/19.
//
#include "gps_init.h"


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

void GpsInit::RosSetup(ros::NodeHandle &nh, ros::NodeHandle &nh_) {
  sub_gps_pos_ = nh.subscribe(
      "/novatel718d/pos", 1, &GpsInit::GpsPosHandler, this
  );
  sub_gps_heading_ = nh.subscribe(
      "/novatel718d/heading", 1, &GpsInit::GpsHeadingHandler, this
  );
  sub_odom_ = nh.subscribe(
      "/golfcar/odom", 1, &GpsInit::CurOdomHandler, this
  );
  sub_manual_ = nh.subscribe(
      "/initialpose", 1, &GpsInit::ManualHandler, this
  );
  sub_lost_ = nh.subscribe(
      "/localization/lost", 1, &GpsInit::LostHandler, this
  );
  sub_init_ = nh.subscribe(
      "/init_location/pose", 1, &GpsInit::InitHandler, this
  );
  pub_gps2odom_ = nh.advertise<nav_msgs::Odometry>(
      "/gps2odom", 1
  );
  pub_odom2gps_ = nh.advertise<sensor_msgs::NavSatFix>(
      "/odom2gps", 1
  );
//  pub_lost_ = nh.advertise<std_msgs::Bool>(
//      "/localization/lost", 1
//  );
}
void GpsInit::Process() {
  if (has_pos_) {
    auto ros_time_diff = gps_pos_time_ - gps_heading_time_;
    double time_diff = ros_time_diff.toSec();

    Eigen::Isometry3d e;
    e.setIdentity();
    if (has_heading_ && std::fabs(time_diff) < 0.5) {
      e.prerotate(gps_heading_);
    } else {
      e.prerotate(Eigen::Quaterniond::Identity());
    }
    e.pretranslate(gps_pos_);
    nav_msgs::Odometry m;
    m.header.stamp = gps_pos_time_;
    m.header.frame_id = "map";
    m.pose.covariance = gps_covariance_;
    tf::poseEigenToMsg(e, m.pose.pose);
    pub_gps2odom_.publish(m);
    if (status_ == STATUS::LOST_WITHOUT_GPS) {
      status_ = STATUS::LOST_WITH_GPS;
    }

    has_pos_ = false;
    has_heading_ = false;
  }
}
void GpsInit::GpsPosHandler(const sensor_msgs::NavSatFix::ConstPtr &gps_pos) {

    ROS_INFO("GET GPSPOSE HANDLER...");

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
    ROS_INFO("GET GPSHEADING HANDLER...");
    auto q = gps_heading->quaternion;
  if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w))
    return;
  gps_heading_time_ = gps_heading->header.stamp;
  gps_heading_ = Eigen::Quaterniond(q.z, 0, 0, q.w); // gps heading to map
  has_heading_ = true;
}

void GpsInit::ManualHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_estimate_msgs) {
  for (int i = 0; i < 5; i++) {
    ros::Rate(1).sleep();
  }
}

void GpsInit::CurOdomHandler(const nav_msgs::Odometry::ConstPtr &odom) {
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

  pub_odom2gps_.publish(msgs);
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

