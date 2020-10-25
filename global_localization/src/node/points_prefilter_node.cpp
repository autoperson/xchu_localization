//
// Created by xchu on 2020/7/1.
//

#include "global_localization/points_prefilter.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "points_prefilter_node");
    ROS_INFO("\033[1;32m---->\033[0m Points Prefilter Started.");
    global_localization::PointsPrefilter filter;

    ros::spin();
    return 0;
}