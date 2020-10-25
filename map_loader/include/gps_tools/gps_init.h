#ifndef POINT_LOCALIZATION_GPS_INIT_H
#define POINT_LOCALIZATION_GPS_INIT_H

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518    ///< WGS84 MINOR AXIS

class GpsInit {
 public:
  GpsInit();
  explicit GpsInit(std::string origin);
  void RosSetup(ros::NodeHandle &nh, ros::NodeHandle &nh_);
  void Process();

    Eigen::Vector3d lla_origin_;
    bool has_pos_;
    bool has_heading_;

private:
  void GpsPosHandler(const sensor_msgs::NavSatFix::ConstPtr &gps_pos);
  void GpsHeadingHandler(const geometry_msgs::QuaternionStamped::ConstPtr &gps_heading);
  void ManualHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_estimate_msgs);
  void CurOdomHandler(const nav_msgs::Odometry::ConstPtr &odom);
  void LostHandler(const std_msgs::Bool::ConstPtr &lost);
  void InitHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init);

  static Eigen::Vector3d GpsMsg2Eigen(const sensor_msgs::NavSatFix &gps_msgs);
  static Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla);
  static Eigen::Vector3d ECEF2LLA(const Eigen::Vector3d &ecef);
  Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef);
  Eigen::Vector3d ENU2ECEF(const Eigen::Vector3d &enu);

 private:
  ros::Subscriber sub_gps_pos_;
  ros::Subscriber sub_gps_heading_;
  ros::Subscriber sub_manual_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_lost_;
  ros::Subscriber sub_init_;
  ros::Publisher pub_gps2odom_;
  ros::Publisher pub_odom2gps_;
  ros::Publisher pub_lost_;

  ros::Time gps_pos_time_;
  Eigen::Vector3d gps_pos_;
  ros::Time gps_heading_time_;
  Eigen::Quaterniond gps_heading_;
  boost::array<double, 36> gps_covariance_;

  Eigen::Vector3d curr_odom_pos_;



  char status_;

};

#endif //POINT_LOCALIZATION_GPS_INIT_H
