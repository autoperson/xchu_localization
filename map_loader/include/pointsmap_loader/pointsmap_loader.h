//
// Created by xchu on 2020/5/19.
//

#ifndef SRC_HDMAP_SERVICE_H
#define SRC_HDMAP_SERVICE_H

//  引入外部头文件
#include "gps_tools/gpsTools.h"
//  ros
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
//  tf
#include <tf/transform_listener.h>
//   pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>


static gpsTools gpsTools;

namespace pointsmap_loader {

    using PointT = pcl::PointXYZI;

    class PointsMapLoader {
    public:
        explicit PointsMapLoader(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

        void ParamInitial(const ros::NodeHandle &nh, const ros::NodeHandle &private_handle);

        void LocalmapCallback(/*const ros::TimerEvent &event*/);

        void GnssCallback(const sensor_msgs::NavSatFixPtr &gps_msg);

        void MainLoop();

    private:

        ros::NodeHandle nh_, private_nh_;

        // ROS
        ros::Publisher globalmap_pub, localmap_pub;
        pcl::PointCloud<PointT>::Ptr globalmap;


        Eigen::Vector3d gps_pos_;
        geometry_msgs::PoseStampedPtr curr_pose;
        bool has_pos_ = false;
        double origin_latitude, origin_longitude, origin_altitude;

        std::string pcd_path;
        std::string gps_topic;
        double downsample_resolution = 0.5;
        int circle_radius = 150;
        int mapUpdateTime = 5;

        ros::Timer timer;

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

        double lidar_height;
        double trim_low;
        double trim_high;
    };

} // namespace hdmap_service


#endif //SRC_HDMAP_SERVICE_H
