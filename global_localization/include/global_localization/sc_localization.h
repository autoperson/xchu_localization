//
// Created by xchu on 2020/7/3.
//

#ifndef SRC_SC_LOCALIZATION_H
#define SRC_SC_LOCALIZATION_H

// 外部头文件
#include "scan_context/Scancontext.h"
#include "gps_tools/gpsTools.h"
//#include "common.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <iostream>



namespace global_localization {
    // corresponding to g2o
    struct PointWithTf {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d loc;
        Eigen::Quaterniond tf;

        PointWithTf(double x, double y, double z,
                    double qx, double qy, double qz, double qw) :
                loc(x, y, z),
                tf(qw, qx, qy, qz) {
        }
    };// 读取文件夹内的点云文件，并排序

    class SCLocalization {
    public:
        SCLocalization();

        void getAllFiles(std::string path, std::vector<std::string> &files);

        void LoadG2O(const std::string &g2o_file, std::vector<PointWithTf> &g2o_path_);

        void WriteFile(const std::string &path, std::vector<Eigen::MatrixXd> &sc);

        void PublishOdometry(const ros::Time &time,
                             const ros::Publisher &topic_pub,
                             const std::string &base_frame_id,
                             const std::string &child_frame_id,
                             const Eigen::Matrix4f &transform_matrix);

    private:
        void ParamInitial();

        void PointsCallback(const sensor_msgs::PointCloud2 &msg);

        void GnssCallback(const sensor_msgs::NavSatFix &msg);


        pcl::PointCloud<pcl::PointXYZI> scan, filtered_scan, transformed_scan;// crop scan


        ros::NodeHandle nh_;
        ros::Subscriber points_sub_;
        ros::Subscriber gps_sub_;

        ros::Publisher aligned_pub_;
        ros::Publisher gnss_odom_pub_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_;

        // 对应的g2o pose
        std::vector<PointWithTf> g2o_pose_vec_;

        // gps_tools
        gpsTools gpsTools_;
        Eigen::Vector3d gps_pose_, prev_pose_;
        double yaw;
        Eigen::Quaternionf init_q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f init_p = Eigen::Vector3f::Identity();
        geometry_msgs::Quaternion _quat;
        double origin_latitude =  22.533571666666667, origin_longitude = 113.93850999999999, origin_altitude = 7.5;// map yuandian

        // sc
        SC2::SCManager scManager;

        //  全局map里面的sc
        std::vector<Eigen::MatrixXd> polarcontexts_vec_;
        std::vector<Eigen::MatrixXd> polarcontext_invkeys_vec_;
        std::vector<Eigen::MatrixXd> polarcontext_vkeys_vec_;
        KeyMat polarcontext_invkeys_mat_vec_;

        //  全局搜索树
        KeyMat polarcontext_invkeys_to_search_;
        std::unique_ptr<InvKeyTree> polarcontext_tree_;
    };

}


#endif //SRC_SC_LOCALIZATION_H
