//
// Created by xchu on 2020/6/19.
//

#include "lidar_localization/lidar_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_localization_node");

  LidarLocalization test;
  test.Spin();

  return 0;
}
