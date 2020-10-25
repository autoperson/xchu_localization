//
// Created by xchu on 2020/6/19.
//

#ifndef SRC_LIDAR_LOCALIZATION_H
#define SRC_LIDAR_LOCALIZATION_H

#include "lidar_localization/pose_estimator.h"

#include <pclomp/ndt_omp.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl_omp_registration/ndt.h>  // if USE_PCL_OPENMP
//#include <pclomp/ndt_omp.h>
//#include <teaser/ply_io.h>
//#include <teaser/registration.h>
//#include "gps_tools/gpsTools.h"
#include <pclomp/ndt_omp.h>
#include "omp.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <chrono>
#include <random>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <deque>
#include <Eigen/Core>
#include <mutex>

static pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> cpu_ndt;  // cpu方式  --可以直接调用吗??可以


class LidarLocalization {
 public:
  explicit LidarLocalization();

  /**
   * 定位算法主流程
   * @return
   */
  int Spin();

 private:

  /**
   * 接收并处理map
   * @param msg
   */
  void GlobalMapCallback(const sensor_msgs::PointCloud2 &msg);

  /**
   * callback 只添加数据
   * @param input
   */
  void PointsCallback(const sensor_msgs::PointCloud2 &input);

  void ImuCallback(const sensor_msgs::ImuConstPtr &msg);
  /**
   * 接收初定位结果
   * @param msg
   */
  void InitialCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

  /**
   * 初始化参数
   */
  void ParamInitial();

  /**
   * ----------------------------------------------------------------
   * 后面的都是工具类
   * ----------------------------------------------------------------
   */
  void PublishPose(const ros::Time &time,
                   const ros::Publisher &topic_name,
                   const std::string &base_frame_id,
                   const Eigen::Matrix4f &transform_matrix
  );

  void PublishTransformedCloud(const ros::Time &time,
                               const ros::Publisher &topic_pub,
                               const std::string &base_frame_id,
                               const Eigen::Matrix4f &matrix,
                               const pcl::PointCloud<pcl::PointXYZI> &scan);

  void PublishOdometry(const ros::Time &stamp, const Eigen::Matrix4f &pose/*, const Eigen::MatrixXf &cov*/);

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const double &downsample_resolution);

  geometry_msgs::TransformStamped
  matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose, const std::string &frame_id,
                   const std::string &child_frame_id);

 private:
  ros::NodeHandle nh_;
  tf::Transform transform;

  //  publisher
  ros::Publisher current_pub;
  ros::Publisher current_odom_pub;
  ros::Publisher aligned_pub;
  ros::Publisher setpose_pub;

  ros::Subscriber map_sub;
  ros::Subscriber points_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber initial_sub;

  //pcl::PointCloud<pcl::PointXYZI> scan, filtered_scan, transformed_scan;// crop scan


  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // imu和cloud队列
  std::mutex mutex_lock;
  std::deque<sensor_msgs::Imu> imuBuf;
  std::deque<sensor_msgs::PointCloud2> cloudBuf;

  tf::TransformBroadcaster pose_broadcaster;
  boost::circular_buffer<double> processing_time;

 public:
  //gpsTools gtools;  //  gps transform
  std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

  double exe_time = 0.0;
  int iteration = 0;
  double fitness_score = 0.0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap;

  int map_points_num = 0; // map
  int scan_points_num = 0; // filter scan

  double origin_latitude, origin_longitude, origin_altitude;// map yuandian
  std::string path, log_path;

  double epsilon, step, res, map_leaf, scan_leaf, scoreThreshold; //ndt
  int iter, align_method;

  int min_scan_range, max_scan_range;// crop pointcloud
  int circle_radius;

  std::string lidar_topic; // topic
  std::string imu_topic;
  std::string heading_topic;
  std::string gps_topic;
  std::string map_topic;

  //init type
  bool has_pose = false;
  bool load_map = false;
  bool pose_success = false;//pose init
  bool init_rviz = false;//pose init
  bool use_imu = false;  //  init param
  bool use_odom = false;  //  init param
  bool invert_imu = false; //

  // init pose
  Eigen::Quaternionf init_q = Eigen::Quaternionf::Identity();
  Eigen::Vector3f init_p = Eigen::Vector3f::Identity();
  Eigen::Matrix4f initial_matrix = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f ndt_trans = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f final_matrix = Eigen::Matrix4f::Identity();

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<PoseEstimator> pose_estimator;
};
//}


#endif //SRC_LIDAR_LOCALIZATION_H
