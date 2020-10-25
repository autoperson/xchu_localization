//
// Created by xchu on 2020/6/15.
//

#ifndef SRC_GLOBAL_LOCALIZATION_H
#define SRC_GLOBAL_LOCALIZATION_H

// 引入外部头文件
//#include "utils/registrations.hpp"
#include "utils/transform.h"
#include "global_localization/cloud_data.hpp"
#include "global_localization/gnss_data.hpp"

#include <pclomp/ndt_omp.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl_omp_registration/ndt.h>  // if USE_PCL_OPENMP
//#include <pclomp/ndt_omp.h>
//#include <teaser/ply_io.h>
//#include <teaser/registration.h>
#include "gps_tools/gpsTools.h"
#include <pclomp/ndt_omp.h>
#include "omp.h"
// ros相关
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
//  pcl相关
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <eigen_conversions/eigen_msg.h>

//  其他
#include <iostream>
#include <chrono>
#include <random>
#include <vector>
#include <mutex>
#include <Eigen/Core>

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
// #define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5

using PointT = pcl::PointXYZI;
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
};

struct NDT_RESULT {
  double exec_time;
  double score;
  int iteration;
  Eigen::Matrix4d matrix;

  NDT_RESULT(double _exec_time, double _score, int _iteration, Eigen::Matrix4d _matrix) {
    exec_time = _exec_time;
    score = _score;
    iteration = _iteration;
    matrix = _matrix;
  }
  NDT_RESULT() {};
};

static pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> cpu_ndt;  // cpu方式  --可以直接调用吗??可以
//static pcl::Registration<PointT, PointT>::Ptr registration;

namespace global_localization {
class GlobalLocalization {
 public:
  GlobalLocalization();

 public:
  gpsTools gtools;  //  gps transform

  bool ValueInitial();

  void MainLoop();
  bool LoadMap(const std::string map_file);
  void LoadG2O(const std::string g2o_file);
  inline double weight(double a, double max_x, double min_y, double max_y, double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

 private:
  void ParamInitial();

  // callback
//        void MapCallback(const sensor_msgs::PointCloud2 &input);
  void LocalmapCallback(const ros::TimerEvent &event);



  bool FindNearestLocation(GNSSData &gnss_data);

  bool GetInitPose(CloudData &cloud_data, GNSSData &gnss_data, Eigen::Matrix4d &matrix);
  void GlobalMapCallback(const sensor_msgs::PointCloud2 &msg);
  void MapCallback(const std::string &path);

  void RvizCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void GnssCallback(const sensor_msgs::NavSatFix &msg);
  void GuessCallback(const nav_msgs::Odometry::ConstPtr &guess_msgs);
  void HeadingCallback(const geometry_msgs::QuaternionStamped &msg);

  void PointsCallback(const sensor_msgs::PointCloud2 &input);
  void IsLostCallback(const std_msgs::Bool::ConstPtr &lost_msgs);
  // utils
//  void CropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &tmp);

//  pcl::PointCloud<pcl::PointXYZI>::Ptr
//  Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const double &downsample_resolution);

  double GetAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est);

  void CaculateBestYaw(Eigen::Matrix4d &out);
  void AlignPoints(const ros::Time &time,
                   const pcl::PointCloud<PointT>::ConstPtr &scan,
                   const Eigen::Matrix4d &guess_matrix, NDT_RESULT &result);

  void AlignPointsCpu(const ros::Time &time,
                      const pcl::PointCloud<PointT>::ConstPtr &scan,
                      const Eigen::Matrix4d &guess_matrix, NDT_RESULT &result);
//  void AlignPoints(const ros::Time &time,
//                   const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan,
//                   const Eigen::Matrix4f &guess_matrix);
//
//  void AlignPointsCpu(const ros::Time &time,
//                      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan,
//                      const Eigen::Matrix4f &guess_matrix);

  /*  bool align_points_teaser(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan,
                             const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &map,
                             const Eigen::Matrix4f &guess_matrix);*/

 private:
  ros::NodeHandle nh_, private_nh_;
  tf::TransformBroadcaster broadcaster;//1.定义一个广播broadcaster
  tf::Transform transform;//2.声明一个变量用来存储转换信息
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

  //  publisher
  ros::Publisher map_pub;
  ros::Publisher final_pub;
  ros::Publisher roimap_pub;
  ros::Publisher init_pub;
  ros::Publisher init_pose_pub_;
  ros::Publisher gnss_odom_pub;
  ros::Publisher aligned_pub;
  ros::Publisher setpose_pub;
  ros::Publisher localmap_pub;
  ros::Publisher pub_gps2odom_;
  ros::Publisher pub_gps2odom2_;

  ros::Subscriber heading_sub;
  ros::Subscriber points_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber gps_sub_;
  ros::Subscriber rviz_sub;
  ros::Subscriber is_lost_sub_;
  ros::Subscriber map_sub;
  pcl::PointCloud<pcl::PointXYZI> scan, filtered_scan, transformed_scan;// crop scan
  pcl::PointCloud<pcl::PointXYZI> final_scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tempor_cloud;

  std::mutex cloud_buff_mutex_; // 线程锁
  std::deque<CloudData> new_cloud_data_;
  std::deque<CloudData> cloud_data_buff;

  std::mutex gnss_buff_mutex_;
  std::deque<GNSSData> new_gnss_data_;
  std::deque<GNSSData> gnss_data_buff;

  bool has_guess_;
  bool manual_;
  bool has_odom_when_lost_;
  bool is_lost_;

  Eigen::Isometry3d odom_when_lost_;

 public:
  pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;

  float lidar_height = 2.4, trim_low = 0, trim_high = 5, radius = 100.0;

  int map_points_num = 0; // map
  int scan_points_num = 0; // filter scan

  double origin_latitude, origin_longitude, origin_altitude;// map yuandian
  std::string log_path;

  std::string origin_name, map_dir, map_name, g2o_name; // map的路径
  std::vector<PointWithTf> g2o_path_;

  double epsilon, step, res, map_leaf, scan_leaf, scoreThreshold;
  int iterThreshold; //ndt
  int iter, align_method;

  std::string lidar_topic; // topic
  std::string imu_topic;
  std::string heading_topic;
  std::string gps_topic;
  std::string map_topic;

  //init type
  bool init_rviz = false;
  bool has_pose = false;
  bool load_map = false;
  bool pose_success = false;//pose init

  std::string map_path;
  std::string g2o_path;
  // init pose
//  Eigen::Vector3f init_p = Eigen::Vector3f::Identity();
//  Eigen::Quaternionf rviz_q = Eigen::Quaternionf::Identity();
//  Eigen::Vector3f rviz_p = Eigen::Vector3f::Identity();

  //Eigen::Vector3d gps_pos_;
  Eigen::Vector3d prev_pos_;

  geometry_msgs::Quaternion _quat;
  double yaw;

  // init yaw_vec
  std::vector<double> yaw_vec;

  int cover;
  std::vector<int> iteration_vec;
  std::vector<double> score_vec;
  std::vector<Eigen::Matrix4d> matrix_vec;
  std::vector<Eigen::Matrix4d> testMatrix_vec;

  bool first_scan = false;
  int code = -1;
  ros::Time gps_pos_time_;
  Eigen::Vector3d gps_pos_;
  ros::Time gps_heading_time_;
  Eigen::Quaterniond gps_heading_;
  boost::array<double, 36> gps_covariance_;

  bool has_pos_ = false;
  bool has_heading_ = false;

  char status_;
  Eigen::Matrix4d gnss_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d gnss_matrix2 = Eigen::Matrix4d::Identity();
  bool _orientation_ready = false;

  int gps_i = 0, gps_all = 0, heading_i = 0;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;

  double min_var_x, max_var_x, min_var_q, max_var_q;
};
}

#endif //SRC_GLOBAL_LOCALIZATION_H
