//
// Created by xchu on 2020/7/1.
//

#ifndef SRC_POINTS_PREFILTER_H
#define SRC_POINTS_PREFILTER_H

// ros
#include "ros/ros.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//  pcl相关
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>


namespace global_localization {
    class PointsPrefilter {

    public:

        PointsPrefilter();

    private:
//        typedef pcl::PointXYZI PointT;//

        void ParamInitial();

        void PointsCallback(const sensor_msgs::PointCloud2 &input);

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr
        DistanceFilter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const;

        boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>
        Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const;

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr
        OutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const;

        ros::NodeHandle nh_;

        ros::Subscriber points_sub;
        ros::Subscriber gps_sub;

        ros::Publisher points_pub;

        std::string points_topic;

        bool use_distance_filter;
        double distance_near_thresh;
        double distance_far_thresh;

        pcl::Filter<pcl::PointXYZI>::Ptr downsample_filter;
        pcl::Filter<pcl::PointXYZI>::Ptr outlier_removal_filter;

    };
}


#endif //SRC_POINTS_PREFILTER_H
