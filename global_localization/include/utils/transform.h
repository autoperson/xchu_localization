/**
 * Author: xc Hu
 * Date: 2020/7/14 下午5:16
 * Content: publish点云的一些工具类
 * Email: 2022087641@qq.com
 */
#ifndef SRC_POINT_LOCALIZATION_INCLUDE_UTILS_TRANSFORM_UTILS_H_
#define SRC_POINT_LOCALIZATION_INCLUDE_UTILS_TRANSFORM_UTILS_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/passthrough.h"
namespace utils {

static geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4d &pose, const std::string &frame_id,
                 const std::string &child_frame_id) {
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr &odom_msg) {
  const auto &orientation = odom_msg->pose.pose.orientation;
  const auto &position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

static void PublishOdometry(const ros::Time &time,
                            const ros::Publisher &topic_pub,
                            const std::string &base_frame_id,
                            const std::string &child_frame_id,
                            const Eigen::Matrix4d &transform_matrix) {
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp = time;
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;

  //set the position
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaterniond q;
  q = transform_matrix.block<3, 3>(0, 0);
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  topic_pub.publish(odometry_);
}

static void PublishPose(const ros::Time &time,
                        const ros::Publisher &topic_pub,
                        const std::string &base_frame_id,
                        const Eigen::Matrix4d &transform_matrix) {
  geometry_msgs::PoseWithCovarianceStamped pose_;
  pose_.header.stamp = time;
  pose_.header.frame_id = base_frame_id;

  //set the position
  pose_.pose.pose.position.x = transform_matrix(0, 3);
  pose_.pose.pose.position.y = transform_matrix(1, 3);
  pose_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaterniond q;
  q = transform_matrix.block<3, 3>(0, 0);
  pose_.pose.pose.orientation.x = q.x();
  pose_.pose.pose.orientation.y = q.y();
  pose_.pose.pose.orientation.z = q.z();
  pose_.pose.pose.orientation.w = q.w();

  topic_pub.publish(pose_);
}

static void PublishTransformedCloud(const ros::Time &time,
                                    const ros::Publisher &topic_pub,
                                    const std::string &base_frame_id,
                                    const Eigen::Matrix4d &matrix,
                                    const pcl::PointCloud<pcl::PointXYZI> &scan) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(scan, *aligned_cloud, matrix);
  sensor_msgs::PointCloud2 fScan;
  pcl::toROSMsg(*aligned_cloud, fScan);

  fScan.header.stamp = time;
  fScan.header.frame_id = base_frame_id;
  topic_pub.publish(fScan);
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr
Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud,
           const double &downsample_resolution) {
  boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  voxelgrid->setInputCloud(cloud);
  voxelgrid->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

static void CropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &tmp,
                      const int circle_radius,
                      const Eigen::Vector3d pose_) {
  pcl::PointIndices::Ptr tokeep(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZI> filter(true);

  for (int p = 0; p < (*tmp).size(); p++) {
    pcl::PointXYZI pt;
    pt.x = (double) tmp->points[p].x;
    pt.y = (double) tmp->points[p].y;
    pt.z = (double) tmp->points[p].z;
    pt.intensity = (double) tmp->points[p].intensity;

    float r = circle_radius;
    if (std::pow(pt.x - pose_(0), 2) + std::pow(pt.y - pose_(1), 2) < std::pow(r, 2)) {
      tokeep->indices.push_back(p);
    }
  }

  filter.setInputCloud(tmp);
  filter.setIndices(tokeep);
  filter.filter(*tmp);
}
static void PassThrough(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float x,
                        float y, float limit) {
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterLimits(x - limit, x + limit);
  pass.setFilterFieldName("x");
  pass.filter(*cloud);
  pass.setFilterLimits(y - limit, y + limit);
  pass.setFilterFieldName("y");
  pass.filter(*cloud);
  //  pass.setFilterLimits(-1, FLT_MAX);
  //  pass.setFilterFieldName("z");
  pass.filter(*cloud);
}

static void Isometry2Matrix(Eigen::Isometry3d &in, Eigen::Matrix4d &out) {
  Eigen::Vector3d p = in.matrix().block<3, 1>(0, 3);
  Eigen::Quaterniond q;
  q = in.matrix().block<3, 3>(0, 0);

  out.block<3, 3>(0, 0) = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
  out.block<3, 1>(0, 3) = p.matrix();
}

static void Matrix2Isometry(Eigen::Matrix4d &in, Eigen::Isometry3d &out) {
  Eigen::Quaterniond q;
  q = in.block<3, 3>(0, 0);
  Eigen::Vector3d p = in.block<3, 1>(0, 3);

  out.matrix().block(0, 3, 3, 1) << p;
  out.matrix().block(0, 0, 3, 3) << q.toRotationMatrix();
}
/**
 * @brief voxel filter
 *
 * @param[in/out] cloud the could need to filter
 * @param[in] leaf the leaf size
 */
static void VoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float leaf) {
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZI> voxel_cloud;
  voxel_filter.filter(voxel_cloud);
  *cloud = voxel_cloud;
}
}
#endif //SRC_POINT_LOCALIZATION_INCLUDE_UTILS_TRANSFORM_UTILS_H_
