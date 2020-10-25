//
// Created by xchu on 2020/7/3.
//

#include "global_localization/sc_localization.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "sc_localization_node");
    ROS_INFO("\033[1;32m---->\033[0m SC Localization Started.");
    global_localization::SCLocalization sCLocalization;

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}