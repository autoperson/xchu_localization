#include <ros/ros.h>

#include <iostream>
#include "global_localization/gps_init.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "gps_init");
  ROS_INFO("\033[1;32m---->\033[0m Gps Init Node Started.");

  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  if (argc != 3) {
    ROS_ERROR("Need origin.txt");
    return -1;
  }

  std::cout << "path: " << argv[2] << std::endl;
  GpsInit gps2map(argv[1]);
  gps2map.RosSetup(nh, nh_);

  if (!gps2map.LoadMap(argv[2])) {
    return -1;
  }

  while (ros::ok()) {
    ros::Rate(20).sleep();
    ros::spinOnce();
    gps2map.Process();
  }
  return 0;
}