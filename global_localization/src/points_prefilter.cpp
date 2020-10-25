//
// Created by xchu on 2020/7/1.
//

#include "global_localization/points_prefilter.h"

namespace global_localization {
    PointsPrefilter::PointsPrefilter() : nh_("~") {
        //  参数初始化
        ParamInitial();

        points_sub = nh_.subscribe(points_topic.c_str(), 64, &PointsPrefilter::PointsCallback, this);
        points_pub = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
    }

    void PointsPrefilter::ParamInitial() {
        use_distance_filter = nh_.param<bool>("use_distance_filter", true);
        distance_near_thresh = nh_.param<double>("distance_near_thresh", 1.0);
        distance_far_thresh = nh_.param<double>("distance_far_thresh", 100.0);

        // base_link_frame = nh_.param<std::string>("base_link_frame", "base_link");
        points_topic = nh_.param<std::string>("points_topic", "");

        std::string downsample_method = nh_.param<std::string>("downsample_method", "VOXELGRID");
        double downsample_resolution = nh_.param<double>("downsample_resolution", 0.1);

        //  下采样方法，VOXELGRID、APPROX_VOXELGRID，后者精度稍低但速度更快
        if (downsample_method == "VOXELGRID") {
            //std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
            boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
            voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            downsample_filter = voxelgrid;
        } else if (downsample_method == "APPROX_VOXELGRID") {
            //std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
            boost::shared_ptr<pcl::ApproximateVoxelGrid<pcl::PointXYZI>> approx_voxelgrid(
                    new pcl::ApproximateVoxelGrid<pcl::PointXYZI>());
            approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            downsample_filter = approx_voxelgrid;
        } else {
            if (downsample_method != "NONE") {
                std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
                std::cerr << "       : use passthrough filter" << std::endl;
            }
            std::cout << "downsample: NONE" << std::endl;
        }

        //  根据分布或者临近距离去除离群点
        std::string outlier_removal_method = nh_.param<std::string>("outlier_removal_method", "STATISTICAL");
        if (outlier_removal_method == "STATISTICAL") {
            int mean_k = nh_.param<int>("statistical_mean_k", 20);
            double stddev_mul_thresh = nh_.param<double>("statistical_stddev", 1.0);
//            std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

            pcl::StatisticalOutlierRemoval<pcl::PointXYZI>::Ptr sor(
                    new pcl::StatisticalOutlierRemoval<pcl::PointXYZI>());
            sor->setMeanK(mean_k);
            sor->setStddevMulThresh(stddev_mul_thresh);
            outlier_removal_filter = sor;
        } else if (outlier_removal_method == "RADIUS") {
            double radius = nh_.param<double>("radius_radius", 0.8);
            int min_neighbors = nh_.param<int>("radius_min_neighbors", 2);
//            std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

            pcl::RadiusOutlierRemoval<pcl::PointXYZI>::Ptr rad(new pcl::RadiusOutlierRemoval<pcl::PointXYZI>());
            rad->setRadiusSearch(radius);
            rad->setMinNeighborsInRadius(min_neighbors);
        } else {
            std::cout << "outlier_removal: NONE" << std::endl;
        }
    }



    void PointsPrefilter::PointsCallback(const sensor_msgs::PointCloud2 &input) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(input, *scan);
        if (scan->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr filtered = DistanceFilter(scan);
        filtered = Downsample(filtered);
        filtered = OutlierRemoval(filtered);

        sensor_msgs::PointCloud2 fScan;
        pcl::toROSMsg(*filtered, fScan);
        fScan.header.stamp = input.header.stamp;
        fScan.header.frame_id = input.header.frame_id;

        points_pub.publish(fScan);
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr
    PointsPrefilter::Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
        if (!downsample_filter) {
            return cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        downsample_filter->setInputCloud(cloud);
        downsample_filter->filter(*filtered);
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr
    PointsPrefilter::OutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
        if (!outlier_removal_filter) {
            return cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        outlier_removal_filter->setInputCloud(cloud);
        outlier_removal_filter->filter(*filtered);
        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr
    PointsPrefilter::DistanceFilter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(cloud->size());

        std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
                     [&](const pcl::PointXYZI &p) {
                         double d = p.getVector3fMap().norm();
                         return d > distance_near_thresh && d < distance_far_thresh;
                     }
        );

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;

        filtered->header = cloud->header;

        return filtered;
    }
}