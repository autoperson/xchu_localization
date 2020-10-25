//
// Created by xchu on 2020/6/19.
//

#include "lidar_localization/lidar_localization.h"

LidarLocalization::LidarLocalization() : nh_("~") {
  //  初始化参数
  ParamInitial();

  // 写在构造函数里面,current_pub和xx_sub需要是全局变量
  current_pub = nh_.advertise<sensor_msgs::PointCloud2>("/current_points", 1);
  current_odom_pub = nh_.advertise<nav_msgs::Odometry>("/current_odom", 1);

  initial_sub = nh_.subscribe("/init_location/pose", 8, &LidarLocalization::InitialCallback, this);
  map_sub = nh_.subscribe("/points_map", 1, &LidarLocalization::GlobalMapCallback, this);
  points_sub = nh_.subscribe(lidar_topic.c_str(), 10, &LidarLocalization::PointsCallback, this);
  //if (use_imu) {
  ROS_INFO("Enable Imu_based Prediction !!!");   // 100hz
  imu_sub = nh_.subscribe(imu_topic.c_str(), 400, &LidarLocalization::ImuCallback, this);
  //}

}

int LidarLocalization::Spin() {
//  std::deque<sensor_msgs::Imu> imu_data_buff;
//  std::deque<sensor_msgs::PointCloud2> cloud_data_buff;

  ros::Rate rate(10);
  //  ros::MultiThreadedSpinner spinner(4);
  while (ros::ok()) {
    ros::spinOnce();
//    rate.sleep();
    //ProcessCloud();
    //    spinner.spin();
    // rate.sleep();

    // 往buffer中装数据
//    mutex_lock.lock();
//    if (imuBuf.size() > 0) {
//      imu_data_buff.insert(imu_data_buff.end(), imuBuf.begin(), imuBuf.end());
//      imuBuf.clear();
//    }
//    if (cloudBuf.size() > 0) {
//      cloud_data_buff.insert(cloud_data_buff.end(), cloudBuf.begin(), cloudBuf.end());
//      cloudBuf.clear();
//    }
//    mutex_lock.unlock();
//
//    // 不管了，直接默认使用imu
//    if (imu_data_buff.size() == 0) {
//      ROS_ERROR("NO iMU data...");
//    }

//    double d_time = cloud_data.time - imu_data.time;
//    if (d_time < -0.05) {
//      cloud_data_buff.pop_front();
//    } else if (d_time > 0.05) {
//      imu_data_buff.pop_front();
//      gnss_data_buff.pop_front();
//    } else {
//      cloud_data_buff.pop_front();
//      imu_data_buff.pop_front();
//      gnss_data_buff.pop_front();
//    }

    if (!cloudBuf.empty()) {
      // 点云的一些操作
      pcl::PointCloud<pcl::PointXYZI> scan, filtered_scan, transformed_scan;
      mutex_lock.lock();
      pcl::fromROSMsg(cloudBuf.front(), scan);
      ros::Time stamp = (cloudBuf.front()).header.stamp;
      cloudBuf.pop_front(); // pop掉
      mutex_lock.unlock();
      // 取queue里面的第一条数据

      if (scan.empty()) {
        ROS_ERROR("Scan is empty !!!");
        continue;
      }
      std::vector<int> indices;   //remove NAN
      pcl::removeNaNFromPointCloud(scan, scan, indices);

      // 初始化预测
      if (!use_imu) {
        pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
      } else {

//        sensor_msgs::Imu imu_msg = imuBuf.front();
//        const auto imu_time = imu_msg.header.stamp;
//        imuBuf.pop_front();

//        const auto &acc = imu_msg.linear_acceleration;
//        const auto &gyro = imu_msg.angular_velocity;
//        double gyro_sign = invert_imu ? -1.0 : 1.0;
//        pose_estimator->predict(imu_time,
//                                Eigen::Vector3f(acc.x, acc.y, acc.z),
//                                gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        ROS_INFO("Received IMU acceleration");

        std::lock_guard<std::mutex> lock(imu_data_mutex);
        auto imu_iter = imu_data.begin();
        for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
          if (stamp < (*imu_iter)->header.stamp) {
            break;
          }
          const auto &acc = (*imu_iter)->linear_acceleration; // 线性加速度
          const auto &gyro = (*imu_iter)->angular_velocity; // 角速度
          double gyro_sign = invert_imu ? -1.0 : 1.0;
          pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z),
                                  gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }
        imu_data.erase(imu_data.begin(), imu_iter);
      }

      for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++) {
        pcl::PointXYZI p;
        p.x = (double) item->x;
        p.y = (double) item->y;
        p.z = (double) item->z;
        p.intensity = (double) item->intensity;
        double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (2 < r && r < 75) {
          filtered_scan.push_back(p);
        }
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filtered_scan));
      auto filtered = Downsample(transformed_scan_ptr, 0.5);
      scan_points_num = filtered->size();

      // 修正
      auto t1 = ros::WallTime::now();
      auto aligned = pose_estimator->correct(filtered);
      auto t2 = ros::WallTime::now();

      processing_time.push_back((t2 - t1).toSec());
      if (current_pub.getNumSubscribers()) {
        aligned->header.frame_id = "map";
        aligned->header.stamp = transformed_scan_ptr->header.stamp;
        current_pub.publish(aligned);
      }
      PublishOdometry(stamp, pose_estimator->matrix());

      iteration = registration->getFinalNumIteration();
      ndt_trans = registration->getFinalTransformation();
      fitness_score = registration->getFitnessScore();
      final_matrix = ndt_trans;

      std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
      std::cout << "Number of Filtered Map Points: " << map_points_num << " points." << std::endl;
      std::cout << "Fitness Score: " << fitness_score << std::endl;
      std::cout << "Number of Iterations: " << iteration << std::endl;
      std::cout << "Transformation Matrix: " << std::endl << ndt_trans << std::endl;
      std::cout << "Align time: " << (t2 - t1).toSec() << " s" << std::endl;
      std::cout << "-----------------------------------------------------------------" << std::endl;
    }
  }
  if (!ros::ok()) {
    std::cout << "killing progress" << std::endl;
  }
  return 0;
}

void LidarLocalization::ImuCallback(const sensor_msgs::ImuConstPtr &msg) {
  std::lock_guard<std::mutex> lock(imu_data_mutex);
  imu_data.push_back(msg);

//  mutex_lock.lock();
//  imuBuf.push_back(*msg);
//  mutex_lock.unlock();
}

void LidarLocalization::PointsCallback(const sensor_msgs::PointCloud2 &msg) {
//  std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
  if (!pose_estimator) {
    ROS_ERROR("Please initialize pose first ...\n");
    return;
  }
  if (!globalmap) {
    ROS_ERROR("Globalmap has not been received !!!");
    return;
  }

  mutex_lock.lock();
  cloudBuf.push_back(msg);
  mutex_lock.unlock();
}

void LidarLocalization::InitialCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  //  接收初始化信息, pose在系统中以T矩阵的方式去表达
  init_p = Eigen::Vector3f(msg.pose.pose.position.x, msg.pose.pose.position.y,
                           msg.pose.pose.position.z);
  init_q = Eigen::Quaternionf(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  initial_matrix.block<3, 3>(0, 0) = init_q.toRotationMatrix();
  initial_matrix.block<3, 1>(0, 3) = init_p;

  if (initial_matrix != Eigen::Matrix4f::Identity() && !pose_success) {

    pose_success = true;
    pose_estimator.reset(
        new PoseEstimator(registration, ros::Time::now(), init_p, init_q, 0.5));
    ROS_INFO("Received Initial pose!");
    std::cout << "Init Matrix : \n" << initial_matrix << std::endl;
  }
}

void LidarLocalization::GlobalMapCallback(const sensor_msgs::PointCloud2 &msg) {
  ROS_INFO("Received global point cloud map ....");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(msg, *cloud);
  globalmap = cloud;

  globalmap = Downsample(globalmap, 0.5); //  下采样
  map_points_num = globalmap->size();

  if (align_method == 1) {
    ROS_INFO("Set target cloud ....");
    registration->setInputTarget(globalmap); //  设置ndt targetCloud
  } else if (align_method == 0) {
    cpu_ndt.setInputTarget(globalmap);
  }
}

void LidarLocalization::ParamInitial() {
  std::cout << "***************lidar localization***************************" << std::endl;
  nh_.param("scoreThreshold", scoreThreshold, 0.5);
  nh_.param("epsilon", epsilon, 0.01);
  nh_.param("step", step, 0.1);
  nh_.param("iter", iter, 10);
  nh_.param("scan_leaf", scan_leaf, 0.5);
  nh_.param("map_leaf", map_leaf, 1.0);
  nh_.param("res", res, 2.0);
  nh_.param("align_method", align_method, 0);
  nh_.param("min_scan_range", min_scan_range, 2);
  nh_.param("max_scan_range", max_scan_range, 75);
  nh_.param("circle_radius", circle_radius, 150);
  nh_.param<std::string>("path", path, "");
  nh_.param<std::string>("lidar_topic", lidar_topic, "/top/rslidar_points");
  nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
  nh_.param<std::string>("heading_topic", heading_topic, " /novatel718d/heading");
  nh_.param<std::string>("gps_topic", gps_topic, "/novatel718d/pos");
  nh_.param<std::string>("map_topic", map_topic, "/points_map"); //no kongge
  nh_.param<bool>("invert_imu", invert_imu, "true"); //no kongge
  nh_.param<bool>("use_imu", use_imu, "true"); //no kongge
  nh_.param<bool>("use_odom", use_odom, "true"); //no kongge

//  std::cout << "scoreThreshold : " << scoreThreshold << std::endl;
//  //std::cout << "epsilon : " << epsilon << std::endl;
//  //std::cout << "step : " << step << std::endl;
//  std::cout << "iter : " << iter << std::endl;
//  std::cout << "scan leaf : " << scan_leaf << std::endl;
//  std::cout << "map leaf : " << map_leaf << std::endl;
//  std::cout << "res : " << res << std::endl;
//  std::cout << "lidar_topic : " << lidar_topic << std::endl;
//  std::cout << "imu_topic : " << imu_topic << std::endl;
//  std::cout << "gps_topic : " << gps_topic << std::endl;
//  std::cout << "heading_topic : " << heading_topic << std::endl;
//  std::cout << "map_topic : " << map_topic << std::endl;
//  std::cout << "min_scan_range : " << min_scan_range << std::endl;
//  std::cout << "max_scan_range : " << max_scan_range << std::endl;
//  std::cout << "circle_radius : " << circle_radius << std::endl;
//  std::cout << "align_method : " << align_method << std::endl;
//  std::cout << "use_imu : " << use_imu << std::endl;
//  std::cout << "use_odom : " << use_odom << std::endl;
//  std::cout << "invert_imu : " << invert_imu << std::endl;
  std::cout << "***************lidar localization***************************" << std::endl;

  processing_time.resize(16);

  if (align_method == 1) {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    // ndt_omp 特有参数
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(omp_get_num_threads());  //  设置最大线程, 注意需要引入头文件omp.h
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(2.0);
    //ndt->setMaximumIterations(30);
    //ndt->setStepSize(0.1);

    // 此处设置ndt参数之后才能赋值给registration,否则后面无法使用getFinalScore函数
    registration = ndt;
  } else if (align_method == 0) {
    cpu_ndt.setTransformationEpsilon(epsilon);
    cpu_ndt.setStepSize(step);
    cpu_ndt.setResolution(res);
    cpu_ndt.setMaximumIterations(iter);
  } else {
    std::cout << "ndt库选择有误 !!!" << std::endl;
  }

  //  接收初始化信息, pose在系统中以T矩阵的方式去表达
  /*init_p = Eigen::Vector3f(-160.510, -1180.98, 8.26732);
  init_q = Eigen::Quaternionf(0.538914, -0.00186377, -0.00474991, -0.842345);

  initial_matrix.block<3, 3>(0, 0) = init_q.toRotationMatrix();
  initial_matrix.block<3, 1>(0, 3) = init_p;

  if (initial_matrix != Eigen::Matrix4f::Identity() && !pose_success) {
    //  初始矩阵不为0，则初始化成功
    pose_success = true;
    //  这里需要设定初始pose
    pose_estimator.reset(
        new lidar_localization::PoseEstimator(registration, ros::Time::now(), init_p, init_q, 0.5));
    ROS_INFO("Received Init pose!");
    std::cout << "Init Matrix : \n" << initial_matrix << std::endl;
  }*/

}

void LidarLocalization::PublishPose(const ros::Time &time,
                                    const ros::Publisher &topic_pub,
                                    const std::string &base_frame_id,
                                    const Eigen::Matrix4f &transform_matrix) {
  geometry_msgs::PoseWithCovarianceStamped pose_;
  pose_.header.stamp = time;
  pose_.header.frame_id = base_frame_id;

  //set the position
  pose_.pose.pose.position.x = transform_matrix(0, 3);
  pose_.pose.pose.position.y = transform_matrix(1, 3);
  pose_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  pose_.pose.pose.orientation.x = q.x();
  pose_.pose.pose.orientation.y = q.y();
  pose_.pose.pose.orientation.z = q.z();
  pose_.pose.pose.orientation.w = q.w();

  topic_pub.publish(pose_);
}

void LidarLocalization::PublishTransformedCloud(const ros::Time &time,
                                                const ros::Publisher &topic_pub,
                                                const std::string &base_frame_id,
                                                const Eigen::Matrix4f &matrix,
                                                const pcl::PointCloud<pcl::PointXYZI> &scan) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(scan, *aligned_cloud, matrix);
  sensor_msgs::PointCloud2 fScan;
  pcl::toROSMsg(*aligned_cloud, fScan);

  fScan.header.stamp = time;
  fScan.header.frame_id = base_frame_id;
  topic_pub.publish(fScan);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
LidarLocalization::Downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud,
                              const double &downsample_resolution) {
  boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZI>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  voxelgrid->setInputCloud(cloud);
  voxelgrid->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

void LidarLocalization::PublishOdometry(const ros::Time &stamp,
                                        const Eigen::Matrix4f &pose/*, const Eigen::MatrixXf &cov*/) {
  // broadcast the transform over tf
  geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", "base_link");
  pose_broadcaster.sendTransform(odom_trans);

  // publish the transform
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";

  odom.pose.pose.position.x = pose(0, 3);
  odom.pose.pose.position.y = pose(1, 3);
  odom.pose.pose.position.z = pose(2, 3);
  odom.pose.pose.orientation = odom_trans.transform.rotation;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  // odom.pose.covariance[0] = cov;

  current_odom_pub.publish(odom);
  // 0.00924323 -4.77469e-09 -2.49199e-11   0.00872456  -4.6624e-09 -1.81717e-11 -5.62527e-10 -2.59378e-12  1.43473e-11 -1.94124e-09            0            0            0  7.11898e-13 -9.81495e-14  2.46305e-11
  // -4.77566e-09   0.00924582  5.97285e-10 -4.70652e-09   0.00872053  5.91688e-10  5.88578e-11  1.37791e-12  1.35827e-11  4.19073e-11            0            0            0 -7.36171e-13  1.27718e-13 -2.21752e-11
  // -2.49199e-11  5.97091e-10   0.00924358  2.69396e-13   5.7773e-10   0.00872523 -3.55736e-10  8.59898e-13  2.67603e-12  5.25159e-10            0            0            0 -9.92535e-16  2.08194e-15  3.07418e-13
  //   0.00872456  -4.7056e-09  2.69368e-13      1.01086 -2.38955e-08  1.19065e-09 -9.18661e-10  9.07893e-12  3.43695e-11   8.2927e-09            0            0            0  8.93389e-11 -5.07215e-11  2.59073e-09
  // -4.66332e-09   0.00872052  5.77913e-10 -2.38964e-08      1.01102  1.25426e-08 -8.29616e-09  2.95783e-12   1.8762e-11 -2.35448e-09            0            0            0 -7.99873e-11  5.78062e-11 -2.42224e-09
  // -1.81717e-11  5.91504e-10   0.00872523  1.19065e-09  1.25424e-08      1.01086 -8.62535e-12  4.58023e-14 -1.19542e-13 -8.68823e-12            0            0            0 -1.36994e-15  1.00655e-15  2.04317e-13
  // -5.30209e-10  6.19897e-11 -3.43318e-10 -9.15734e-10 -8.35141e-09   3.2032e-12   0.00147907 -9.18285e-07 -3.11676e-06 -0.000336613            0            0            0 -3.14823e-08 -9.26203e-08  -1.6882e-05
  // -2.54902e-12  1.57617e-12  8.28663e-13  8.89383e-12   3.3721e-12  1.65276e-14 -9.18264e-07   0.00107582  7.78377e-09  8.62909e-07            0            0            0 -1.04666e-05 -1.62944e-05  9.34921e-08
  // 1.36554e-11  1.29183e-11  3.05432e-12  3.43603e-11  1.83489e-11  2.37477e-14 -3.11678e-06  7.78389e-09   0.00107585  2.93814e-06            0            0            0  1.62945e-05 -1.04667e-05 -2.50784e-08
  // -1.94078e-09  4.40231e-11  5.33389e-10  8.29336e-09 -2.35248e-09 -7.05588e-13 -0.000336613  8.62896e-07  2.93816e-06   0.00139038            0            0            0 -7.96556e-08  2.16763e-08 -9.22359e-06
  //           0            0            0            0            0            0            0            0            0            0    0.0100102            0            0            0            0            0
  //           0            0            0            0            0            0            0            0            0            0            0    0.0100102            0            0            0            0
  //           0            0            0            0            0            0            0            0            0            0            0            0    0.0100102            0            0            0
  // 7.11207e-13 -7.44959e-13  6.82615e-16  8.93383e-11 -7.99956e-11  2.11294e-16 -3.14828e-08 -1.04666e-05  1.62945e-05 -7.96704e-08            0            0            0   0.00970421  2.50379e-10 -1.70113e-07
  // -9.89885e-14  1.28759e-13   1.9586e-16 -5.07223e-11  5.78071e-11 -7.77022e-16 -9.26277e-08 -1.62944e-05 -1.04667e-05  2.16908e-08            0            0            0  2.50307e-10   0.00970421  1.10418e-07
  // 2.45812e-11 -2.22391e-11  5.19114e-14  2.59068e-09  -2.4223e-09 -3.68625e-14  -1.6882e-05  9.34925e-08 -2.50787e-08 -9.22359e-06            0            0            0 -1.70113e-07  1.10418e-07   0.00969443

}

geometry_msgs::TransformStamped
LidarLocalization::matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose,
                                    const std::string &frame_id,
                                    const std::string &child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
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
