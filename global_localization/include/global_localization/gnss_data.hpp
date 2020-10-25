/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>
#include <nav_msgs/Odometry.h>
//#include "Geocentric/LocalCartesian.hpp"

class GNSSData {
 public:
  ros::Time time;
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  int status = 0;
  int service = 0;
  boost::array<double, 36> gps_covariance;


  Eigen::Vector3d position;
  geometry_msgs::Quaternion orientation;
  Eigen::Matrix4d guess_matrix = Eigen::Matrix4d::Identity();

  nav_msgs::Odometry::Ptr odom;

  static double origin_longitude;
  static double origin_latitude;
  static double origin_altitude;

 private:
  //static GeographicLib::LocalCartesian geo_converter;
  //static bool origin_position_inited;

 public:
//  void InitOriginPosition();
//
//  void UpdateXYZ();
//
//  static bool SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time);
};

#endif