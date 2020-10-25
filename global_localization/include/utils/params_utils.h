/**
 * Author: xc Hu
 * Date: 2020/7/9 下午5:07
 * Content: 参数初始化
 * Email: 2022087641@qq.com
 */
#ifndef SRC_GLOBAL_LOCALIZATION_INCLUDE_UTILS_PARAMS_UTILS_H_
#define SRC_GLOBAL_LOCALIZATION_INCLUDE_UTILS_PARAMS_UTILS_H_

#include "ros/ros.h"

namespace params_utils {

static void ParamInit(ros::NodeHandle &nh_) {

  std::cout << "******************************************" << std::endl;
  nh_.param("scoreThreshold", scoreThreshold, 0.5);
  nh_.param("epsilon", epsilon, 0.01);
  nh_.param("step", step, 0.1);
  nh_.param("iter", iter, 10);
  nh_.param("scan_leaf", scan_leaf, 0.5);
  nh_.param("map_leaf", map_leaf, 1.0);
  nh_.param("res", res, 2.0);
  nh_.param("cover", cover, 40);
  nh_.param("align_method", align_method, 0);
  nh_.param("min_scan_range", min_scan_range, 2);
  nh_.param("max_scan_range", max_scan_range, 75);
  nh_.param("circle_radius", circle_radius, 150);
  nh_.param("origin_longitude", origin_longitude, 114.045642255);
  nh_.param("origin_latitude", origin_latitude, 22.663029715);
  nh_.param("origin_altitude", origin_altitude, 59.620000000000005);
  nh_.param<std::string>("path", path,
                         "/home/xchu/workspace/init_ws/src/map_loader/data/pcd_map/map.pcd");
  nh_.param<std::string>("lidar_topic", lidar_topic, "/top/rslidar_points");
  nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
  nh_.param<std::string>("heading_topic", heading_topic, " /novatel718d/heading");
  nh_.param<std::string>("gps_topic", gps_topic, "/novatel718d/pos");
  nh_.param<std::string>("map_topic", map_topic, "/points_map"); //no kongge
  nh_.param<std::string>("log_path", log_path,
                         " /home/catalina/workspace/yiqing_ws/src/smartcar/location/ndt_location/log/");

  std::cout << "scoreThreshold : " << scoreThreshold << std::endl;
  std::cout << "epsilon : " << epsilon << std::endl;
  std::cout << "step : " << step << std::endl;
  std::cout << "iter : " << iter << std::endl;
  std::cout << "scan leaf : " << scan_leaf << std::endl;
  std::cout << "map leaf : " << map_leaf << std::endl;
  std::cout << "cover : " << cover << std::endl;
  std::cout << "res : " << res << std::endl;
  std::cout << "lidar_topic : " << lidar_topic << std::endl;
  std::cout << "imu_topic : " << imu_topic << std::endl;
  std::cout << "gps_topic : " << gps_topic << std::endl;
  std::cout << "heading_topic : " << heading_topic << std::endl;
  std::cout << "map_topic : " << map_topic << std::endl;
  std::cout << "min_scan_range : " << min_scan_range << std::endl;
  std::cout << "max_scan_range : " << max_scan_range << std::endl;
  std::cout << "circle_radius : " << circle_radius << std::endl;
  std::cout << "origin_longitude : " << origin_longitude << std::endl;
  std::cout << "origin_latitude : " << origin_latitude << std::endl;
  std::cout << "origin_altitude : " << origin_altitude << std::endl;
  std::cout << "align_method : " << align_method << std::endl;
  std::cout << "******************************************" << std::endl;
}

};

#endif //SRC_GLOBAL_LOCALIZATION_INCLUDE_UTILS_PARAMS_UTILS_H_
