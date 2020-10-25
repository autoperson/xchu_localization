/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CloudData {
 public:
  using POINT = pcl::PointXYZI;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData()
      : cloud_ptr(new CLOUD()) {
  }

 public:
  ros::Time time;
  CLOUD_PTR cloud_ptr;
};

#endif