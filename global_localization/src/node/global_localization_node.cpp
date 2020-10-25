//
// Created by xchu on 2020/6/15.
//

#include "global_localization/global_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_localization_node");
  ROS_INFO("\033[1;32m---->\033[0m Global Localization Started.");
  global_localization::GlobalLocalization test;

  //  加载地图之类的
  test.map_path = argv[1];
  test.g2o_path = argv[2];
  std::cout << "map: " << test.map_dir << ", g2o_path: " << test.g2o_path << std::endl;
  if (!test.LoadMap(argv[1])) {
    ROS_ERROR("LOAD MAP ERROR\n");
    return -1;
  }
  test.LoadG2O(argv[2]);
  if (!test.ValueInitial()) {
    ROS_ERROR("Load map error..");
    return -1;
  }
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    test.MainLoop();
    ros::Rate(1).sleep();
  }
  return 0;
}