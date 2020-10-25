//
// Created by xchu on 2020/6/15.
//

#include "global_localization/global_localization.h"

namespace global_localization {
GlobalLocalization::GlobalLocalization() : nh_("~") {
  //  初始化参数
  ParamInitial();

  //  publisher
  setpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1);

  //roimap_pub = nh_.advertise<sensor_msgs::PointCloud2>("/roi_map", 1); // roi map
  final_pub = nh_.advertise<sensor_msgs::PointCloud2>("/final_points", 1);
  aligned_pub = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1, false);
  init_pub =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initial_pose", 1);
  map_pub = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
  init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/init_location/pose", 1);
  pub_gps2odom_ = nh_.advertise<nav_msgs::Odometry>("/gps2odom", 1);
  pub_gps2odom2_ = nh_.advertise<nav_msgs::Odometry>("/gps2odom2", 1);

  //  suscriber
  heading_sub = nh_.subscribe(heading_topic, 5, &GlobalLocalization::HeadingCallback, this);
  gps_sub = nh_.subscribe(gps_topic, 10, &GlobalLocalization::GnssCallback, this);

  points_sub = nh_.subscribe(lidar_topic, 100, &GlobalLocalization::PointsCallback, this);
  rviz_sub = nh_.subscribe("/initialpose", 10, &GlobalLocalization::RvizCallback, this);
  //gps_sub_ = nh_.subscribe("/gps2odom", 1, &GlobalLocalization::GuessCallback, this);
  is_lost_sub_ =
      nh_.subscribe("/localization/lost", 10, &GlobalLocalization::IsLostCallback, this); //  初始化定位状态，lost的时候才考虑其他

}

bool GlobalLocalization::LoadMap(const std::string map_file) {
  if (pcl::io::loadPCDFile<PointT>(map_file, *globalmap) == -1) {
    PCL_ERROR("Couldn't read map file\n");
    return false;
  }
  ROS_INFO("origin global map size: %d...", globalmap->size());
  utils::VoxelFilter(globalmap, 0.5);
  if (globalmap->is_dense) {
    ROS_INFO("Published global map");
    ROS_INFO("downsample global map size: %d...", globalmap->size());
    return true;
  } else {
    PCL_ERROR("Invalid map file\n");
    return false;
  }
}

bool GlobalLocalization::FindNearestLocation(GNSSData &gnss_data) {

  std::cout << "before: " << gnss_data.position.transpose() << std::endl;
  //在g2o中搜索最近的点
  std::vector<int> point_id;
  std::vector<float> point_distance;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ gps_point(gnss_data.x, gnss_data.y, 0);

  // 在g2o_path中寻找最近点
  if (!g2o_path_.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : g2o_path_) {
      input_cloud->push_back(pcl::PointXYZ(point.loc.x(), point.loc.y(), 0));
    }
    kdtree.setInputCloud(input_cloud);
    if (kdtree.radiusSearch(gps_point, 20, point_id, point_distance) != 0) {
      std::cout << "find correspondent point[" << point_id[0] << "] in g2o file." << std::endl;
      gnss_data.position.z() = gnss_data.z = g2o_path_.at(point_id[0]).loc.z();
      gnss_data.guess_matrix.block<3, 1>(0, 3) = gnss_data.position.matrix();
      std::cout << "after: " << gnss_data.position.transpose() << std::endl;
      return true;
    }
  }
  return false;
}

void GlobalLocalization::LoadG2O(const std::string g2o_file) {
  std::ifstream fin(g2o_file);
  if (!fin) {
    ROS_INFO("[init_location] RUNNING WITHOUT G2O FILE");
    g2o_path_.clear();
    return;
  }

  while (!fin.eof()) {
    std::string name;
    fin >> name;
    if (name == "VERTEX_SE3:QUAT") {
      int index;
      float x, y, z, qx, qy, qz, qw;
      fin >> index >> x >> y >> z >> qx >> qy >> qz >> qw;
      g2o_path_.emplace_back(PointWithTf(x, y, z, qx, qy, qz, qw));
    } else {
      continue;
    }
    if (!fin.good()) break;
  }
  //std::cout << "g2o file: " << g2o_path_.size() << std::endl;
  ROS_INFO("[init_location] RUNNING WITH G2O FILE CONTAINING %lu VERTICES",
           g2o_path_.size());
}

void GlobalLocalization::ParamInitial() {
  std::cout << "******************global_localization************************" << std::endl;
  nh_.param("scoreThreshold", scoreThreshold, 0.5);
  nh_.param("iterThreshold", iterThreshold, 30);
  nh_.param("epsilon", epsilon, 0.01);
  nh_.param("step", step, 0.1);
  nh_.param("iter", iter, 10);
  nh_.param("scan_leaf", scan_leaf, 0.5);
  nh_.param("map_leaf", map_leaf, 1.0);
  nh_.param("res", res, 2.0);
  nh_.param("cover", cover, 40);
  nh_.param("align_method", align_method, 0);
//  nh_.param("min_scan_range", min_scan_range, 2);
//  nh_.param("max_scan_range", max_scan_range, 75);
//  nh_.param("circle_radius", circle_radius, 150);
  nh_.param("origin_longitude", origin_longitude, 114.045642255);
  nh_.param("origin_latitude", origin_latitude, 22.663029715);
  nh_.param("origin_altitude", origin_altitude, 59.620000000000005);
  //nh_.param<std::string>("path", map_dir, "");
//  nh_.param<std::string>("map", map_name, "");
//  nh_.param<std::string>("g2o", g2o_name, "");
//  nh_.param<std::string>("origin", origin_name, "");

  nh_.param<std::string>("lidar_topic", lidar_topic, "/top/rslidar_points");
  nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
  nh_.param<std::string>("heading_topic", heading_topic, " /novatel718d/heading");
  nh_.param<std::string>("gps_topic", gps_topic, "/novatel718d/pos");
  nh_.param<std::string>("map_topic", map_topic, "/points_map"); //no kongge
  nh_.param<std::string>("log_path", log_path, "");
  //nh_.param<std::string>("map_path", map_path, "");

  var_gain_a = nh_.param<double>("var_gain_a", 1.0);
  min_stddev_x = nh_.param<double>("min_stddev_x", 0.1);
  max_stddev_x = nh_.param<double>("max_stddev_x", 5.0);
  min_stddev_q = nh_.param<double>("min_stddev_q", 0.05);
  max_stddev_q = nh_.param<double>("max_stddev_q", 0.2);
  fitness_score_thresh = nh_.param<double>("fitness_score_thresh", 2.0);

  min_var_x = std::pow(min_stddev_x, 2);
  max_var_x = std::pow(max_stddev_x, 2);
  min_var_q = std::pow(min_stddev_q, 2);
  max_var_q = std::pow(max_stddev_q, 2);

  std::cout << "scoreThreshold : " << scoreThreshold << std::endl;
  std::cout << "iterThreshold : " << iterThreshold << std::endl;
  std::cout << "epsilon : " << epsilon << std::endl;
  std::cout << "step : " << step << std::endl;
  std::cout << "iter : " << iter << std::endl;
  std::cout << "scan leaf : " << scan_leaf << std::endl;
  std::cout << "map leaf : " << map_leaf << std::endl;
  std::cout << "cover : " << cover << std::endl;
  std::cout << "res : " << res << std::endl;
  std::cout << "lidar_topic : " << lidar_topic << std::endl;
  std::cout << "imu_topic : " << imu_topic << std::endl;
  std::cout << "gps_topic : " << gps_topic << std::endl;
  std::cout << "heading_topic : " << heading_topic << std::endl;
  std::cout << "map_topic : " << map_topic << std::endl;
//  std::cout << "min_scan_range : " << min_scan_range << std::endl;
//  std::cout << "max_scan_range : " << max_scan_range << std::endl;
//  std::cout << "circle_radius : " << circle_radius << std::endl;
  std::cout << "origin_longitude : " << origin_longitude << std::endl;
  std::cout << "origin_latitude : " << origin_latitude << std::endl;
  std::cout << "origin_altitude : " << origin_altitude << std::endl;
//  std::cout << "path : " << map_dir << std::endl;
//  std::cout << "map : " << map_name << std::endl;
//  std::cout << "g2o : " << g2o_name << std::endl;
//  std::cout << "origin_name : " << origin_name << std::endl;
  std::cout << "*******************global_localization***********************" << std::endl;

  globalmap.reset(new pcl::PointCloud<PointT>());
}

bool GlobalLocalization::ValueInitial() {
  //std::string map_path = map_dir + "/" + map_name;
//  std::cout << "map_dir: " << map_path << std::endl;
//  if (!LoadMap(map_path)) {
//    ROS_ERROR("Load Map Failed...");
//    return false;
//  }
//  ROS_INFO("Loade Map success...");
  //LoadG2O(map_dir + "/" + g2o_name);

  //  ndt初始化时扫描角度设置
  for (int i = 0; i < cover; ++i) {   //  cover = 40 - pai
    double yaw_reg = M_PI * (i - cover / 2) / (cover / 2);
    yaw_vec.push_back(yaw_reg);

    geometry_msgs::Quaternion _quat = tf::createQuaternionMsgFromYaw(yaw_reg);
    Eigen::Quaterniond test_q(_quat.w, _quat.x, _quat.y, _quat.z);

    Eigen::Matrix4d test_guess = Eigen::Matrix4d::Identity();
    test_guess.block<3, 3>(0, 0) = test_q.toRotationMatrix();
    test_guess.block<3, 1>(0, 3) = Eigen::Vector3d::Identity().matrix()/*.cast<double>()*/;

    testMatrix_vec.push_back(test_guess);
  }

  //  加载点云地图并设置原点经纬度
  gtools.lla_origin_ << origin_latitude, origin_longitude, origin_altitude;

  // 选择点云匹配算法
  // registration = utils::select_registration_method(nh_);
  if (align_method == 1) {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    registration = ndt;
    //  特有参数
    registration->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    registration->setNumThreads(omp_get_num_threads());  //  设置最大线程

    //  ndt通用参数
    registration->setTransformationEpsilon(epsilon);
    registration->setStepSize(step);
    registration->setResolution(res);
    registration->setMaximumIterations(iter);
  } else if (align_method == 0) {
    cpu_ndt.setTransformationEpsilon(epsilon);
    cpu_ndt.setStepSize(step);
    cpu_ndt.setResolution(res);
    cpu_ndt.setMaximumIterations(iter);
  } else {
    std::cout << "ndt库选择有误 !!!" << std::endl;
  }

  // 一些全局变量初始化
  has_guess_ = false;
  manual_ = false;
  is_lost_ = false;
  has_odom_when_lost_ = false;
  odom_when_lost_.setIdentity();

  utils::PublishTransformedCloud(ros::Time::now(), map_pub, "map", Eigen::Matrix4d::Identity(), *globalmap);

  //  有其他节点需要初始化的时候才退出循环
  for (size_t i = 0; i < 50; i++) {
    ros::Rate(10).sleep();
    if (map_pub.getNumSubscribers() > 0
        && init_pub.getNumSubscribers() > 0) {
      break;
    }
  }


  // 这里发布全局地图
//  sensor_msgs::PointCloud2 cloud2_msgs;
//  pcl::toROSMsg(*globalmap, cloud2_msgs);
//  cloud2_msgs.header.frame_id = "map";
//  map_pub.publish(cloud2_msgs);

  return true;
}

void GlobalLocalization::MainLoop() {
  if (cloud_data_buff.size() > 3000) {
    cloud_data_buff.clear();
  }
  if (gnss_data_buff.size() > 300) {
    gnss_data_buff.clear();
  }
  // 更新buffer数据
  cloud_buff_mutex_.lock();
  if (new_cloud_data_.size() > 0) {
    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
  cloud_buff_mutex_.unlock();
  // 更新gnss数据
  gnss_buff_mutex_.lock();
  if (new_gnss_data_.size() > 0) {
    gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }
  gnss_buff_mutex_.unlock();


  // call back中用来接受数据，剩下的可以循环处理的丢在此函数中来处理
  if (/*is_lost_*/!pose_success) {
    // 点云匹配获取初值, 没有gnss和cloud数据也不会进去
    if (cloud_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
      CloudData cloud_data = cloud_data_buff.front();
      GNSSData gnss_data = gnss_data_buff.front();

      if (std::abs(gnss_data.time.toSec() - cloud_data.time.toSec()) > 1) {
        ROS_ERROR("LARGE TIME DIFF...");
        return;
      }
      Eigen::Matrix4d odom_trans = Eigen::Matrix4d::Identity();
      if (GetInitPose(cloud_data, gnss_data, odom_trans)) {
        //std::cout << "odom matrix: " << odom_trans << std::endl;
        // 这里初始化成功
        utils::PublishPose(cloud_data.time, init_pose_pub_, "map", odom_trans);
        utils::PublishPose(cloud_data.time, init_pub, "map", odom_trans);
        utils::PublishTransformedCloud(cloud_data.time, final_pub, "map", odom_trans, *(cloud_data.cloud_ptr));
        geometry_msgs::TransformStamped tf_trans = utils::matrix2transform(cloud_data.time, odom_trans, "map",
                                                                           "base_link");
        broadcaster.sendTransform(tf_trans);

        ros::Rate(.5).sleep();
        ros::spinOnce();

        has_odom_when_lost_ = false;
        is_lost_ = false;
        pose_success = true;
      }
    }
  }

}

bool GlobalLocalization::GetInitPose(CloudData &cloud_data, GNSSData &gnss_data, Eigen::Matrix4d &out) {
  if (globalmap->empty()) {
    ROS_INFO("Map cloud is empty");
    return false;
  }
  // downsample cloud map and scan
  tempor_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*globalmap, *tempor_cloud);
//  utils::CropCloud(tempor_cloud, 80, gnss_data.position);
  utils::PassThrough(tempor_cloud, gnss_data.x, gnss_data.y, 100);

  //在g2o中搜索最近的点
  if (!FindNearestLocation(gnss_data)) {
    std::cout << "can not find nearest point..." << std::endl;
  }

  auto filtered_transformed_scan = utils::Downsample(cloud_data.cloud_ptr, 0.5);
  auto filtered_map = utils::Downsample(tempor_cloud, 0.5);
  map_points_num = filtered_map->size();
  scan_points_num = filtered_transformed_scan->size();

  // set target
  if (align_method == 1) {
    registration->setInputTarget(filtered_map); //  设置ndt targetCloud
  } else if (align_method == 0) {
    cpu_ndt.setInputTarget(filtered_map);
  }

  // 先用gnss产生的pose测试
  if (gnss_data.service > 0) {
    if (gnss_data.service == 2) {
      ROS_INFO("Using GNSS Heading...");
      std::cout << "service: " << gnss_data.service << ", matrix : " << std::endl;
      std::cout << gnss_data.guess_matrix << std::endl;
    } else if (gnss_data.service == 1) {
      ROS_INFO("Using GNSS Yaw...");
      std::cout << "service: " << gnss_data.service << ", matrix : " << std::endl;
      std::cout << gnss_data.guess_matrix << std::endl;
    }

    NDT_RESULT result;
    if (align_method == 1) {
      AlignPoints(cloud_data.time, filtered_transformed_scan, gnss_data.guess_matrix, result); // aligned
    } else if (align_method == 0) {
      AlignPointsCpu(cloud_data.time, filtered_transformed_scan, gnss_data.guess_matrix, result); // aligned
    }
    if (result.score < 2 && result.iteration < iter && result.iteration > 0) {
      ROS_INFO("Using heading success...");
      //ROS_INFO("NDT matching success ...");
      std::cout << "Choose score and iteration : " << result.score << ", " << result.iteration
                << std::endl;
      out = result.matrix;
      return true;
    }
    ROS_ERROR("Heading init failed...");
    std::cout << "use heading : " << result.score << ", " << result.iteration << std::endl;
  }
  ROS_ERROR("using scan init...");

  ros::Time time_1 = ros::Time::now(), time_2 = ros::Time::now(), time_3 = ros::Time::now();
//  遍历一圈,获取最佳匹配结果
  for (int j = 0;j < testMatrix_vec.size();++j) {
    testMatrix_vec[j].block<3, 1>(0, 3) = gnss_data.position.matrix();
    if (testMatrix_vec[j] ==Eigen::Matrix4d::Identity() ) {
      continue;
    }

    NDT_RESULT result2;
    if (align_method == 1) {
      AlignPoints(cloud_data.time, filtered_transformed_scan, testMatrix_vec[j], result2); // aligned
    } else if (align_method == 0) {
      AlignPointsCpu(cloud_data.time, filtered_transformed_scan, testMatrix_vec[j], result2); // aligned
    }

// save ndt scores
    matrix_vec.push_back(result2.matrix);
    score_vec. push_back(result2.score);
    iteration_vec.push_back(result2.iteration);

//  如果ndt匹配结果比较好，就直接初始化完成
    if (result2.score < 1 && result2.iteration < iter && result2.iteration > 0) {
      ROS_INFO("NDT matching success ...");
      std::cout << "Choose best score and iteration : " << result2.score << ", " << result2.iteration
                <<
                std::endl;

      out = result2.matrix;
      return true;
    }
  }
  time_2 = ros::Time::now();
  Eigen::Matrix4d final_matrix = Eigen::Matrix4d::Identity();
  CaculateBestYaw(final_matrix);    //  searching best yaw
  time_3 = ros::Time::now();

  std::cout << "1-2: " << (time_2 - time_1) << " s " << "--Total Aligning Time" <<
            std::endl;
  std::cout << "2-3: " << std::setprecision(10) << (time_3 - time_2) << " s " << "--Total Searching Best Yaw"
            <<
            std::endl;

  if (code >= 0) {      //  最小分数的iteration - 最小迭代次数
    out = final_matrix;
    return true;
  }
  ROS_ERROR("Yaw init failed...");
  ROS_ERROR("can not get best init yaw");
  return false;
}

void GlobalLocalization::AlignPoints(const ros::Time &time,
                                     const pcl::PointCloud<PointT>::ConstPtr &scan,
                                     const Eigen::Matrix4d &guess_matrix,
                                     NDT_RESULT &result) {
  //  记录匹配开始时间
  //  默认为格林威治时间至现在的总秒数，不能表示毫秒
  std::chrono::time_point<std::chrono::system_clock> matching_start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> align_start, align_end;
  static double align_time;

  //  ndt配准
  pcl::PointCloud<PointT>::Ptr aligned_cloud(new pcl::PointCloud<PointT>);

  //  tmp点云去和局部点云地图配准
  registration->setInputSource(scan);
  align_start = std::chrono::system_clock::now();
  registration->align(*aligned_cloud, guess_matrix.cast<float>()); //  点云对齐
  align_end = std::chrono::system_clock::now();
  align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

  int iteration = registration->getFinalNumIteration();
  Eigen::Matrix4d ndt_trans = registration->getFinalTransformation().cast<double>(); //  获得ndt匹配结果
  double fitness_score = registration->getFitnessScore();

  std::chrono::time_point<std::chrono::system_clock> matching_end = std::chrono::system_clock::now();
  double exe_time =
      std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() /
          1000.0;

  float w_p = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  NDT_RESULT re(exe_time, fitness_score, iteration, ndt_trans);
  result = re;

  //  发布实时的点云
  utils::PublishTransformedCloud(time, aligned_pub, "map", guess_matrix, *scan);

  std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of Filtered Map Points: " << map_points_num << " points." << std::endl;
  std::cout << "Fitness Score, w_p, w_q: " << fitness_score  << ", " << w_p << ", " << w_q<< std::endl;
  std::cout << "Number of Iterations: " << iteration << std::endl;
  std::cout << "Transformation Matrix: " << std::endl << ndt_trans << std::endl;
  //std::cout << "Align time: " << align_time << "ms" << std::endl;
  std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void GlobalLocalization::AlignPointsCpu(const ros::Time &time,
                                        const pcl::PointCloud<PointT>::ConstPtr &scan,
                                        const Eigen::Matrix4d &guess_matrix, NDT_RESULT &result) {

  std::chrono::time_point<std::chrono::system_clock> matching_start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> align_start, align_end;
  static double align_time;

  //  ndt配准
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(*scan));  // scan保存到scan_ptr中

  //  tmp点云去和局部点云地图配准
  cpu_ndt.setInputSource(scan_ptr);
  align_start = std::chrono::system_clock::now();
  cpu_ndt.align(*aligned_cloud, guess_matrix.cast<float>()); //  点云对齐
  align_end = std::chrono::system_clock::now();
  align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

  int iteration = cpu_ndt.getFinalNumIteration();
  Eigen::Matrix4d ndt_trans = cpu_ndt.getFinalTransformation().cast<double>(); //  获得ndt匹配结果
  double fitness_score = cpu_ndt.getFitnessScore();

  std::chrono::time_point<std::chrono::system_clock> matching_end = std::chrono::system_clock::now();
  double exe_time =
      std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() /
          1000.0;
  float w_p = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  NDT_RESULT re(exe_time, fitness_score, iteration, ndt_trans);
  result = re;
  //  发布实时的点云
  utils::PublishTransformedCloud(time, aligned_pub, "map", guess_matrix, *scan);

  std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of Filtered Map Points: " << map_points_num << " points." << std::endl;
  std::cout << "Fitness Score, w_p, w_q: " << fitness_score  << ", " << w_p << ", " << w_q<< std::endl;
   std::cout << "Number of Iterations: " << iteration << std::endl;
  std::cout << "Transformation Matrix: " << std::endl << ndt_trans << std::endl;
  //std::cout << "Align time: " << align_time << "ms" << std::endl;
  std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void GlobalLocalization::CaculateBestYaw(Eigen::Matrix4d &out) {
  if (score_vec.size() == iteration_vec.size() && score_vec.size() == matrix_vec.size()) {
    //  ROS_INFO("Get score iteration and transform matrix vec successfully.");
    double min_score1 = 9999.0, min_score2 = 9999.0, min_iteration1 = 9999.0, min_iteration2 = 9999.0;
    int score_index1 = 0, iter_index1 = 0, score_index2 = 0, iter_index2 = 0;

    for (int i = 1; i < score_vec.size() - 1; i++) {
      //  cout << "score: " << score_vec[i] << ", " << iteration_vec[i] << "\n" << endl;
      // 全迭代次数都是17咋办 1650 1850
      if (iteration_vec[i] == iter + 2 || iteration_vec[i] == 0/*&& score_vec[i] >= 2*/) {
        continue;
      }
      if (score_vec[i] < min_score1) {
        min_score2 = min_score1;
        score_index2 = score_index1;
        min_score1 = score_vec[i];
        score_index1 = i;
      } else if (score_vec[i] < min_score2) {
        min_score2 = score_vec[i];
        score_index2 = i;
      }
      if (iteration_vec[i] <= min_iteration1) {
        min_iteration2 = min_iteration1;
        iter_index2 = iter_index1;
        min_iteration1 = iteration_vec[i];
        iter_index1 = i;

      } else if (iteration_vec[i] <= min_iteration2) {
        min_iteration2 = iteration_vec[i];
        iter_index2 = i;
      }
    }

    std::cout << "min score: " << min_score1 << ", " << iteration_vec[score_index1] << "; " << min_score2
              << ", " << iteration_vec[score_index2] << std::endl;
    std::cout << "min Iteration: " << score_vec[iter_index1] << ", " << min_iteration1 << "; "
              << score_vec[iter_index2] << ", " << min_iteration2 << std::endl;

    if (min_score1 == 9999.0) {
      std::cout << "score value error ..." << std::endl;
      code = -1;
      return;
    }

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    // 迭代次数和score均为最小
    if (score_index1 == iter_index1 && min_score1 == score_vec[iter_index1] &&
        min_score1 < scoreThreshold) {
      //  score和iter均为最小，直接取为最小值
      std::cout << "Choose best score and iter ..." << std::endl;
      code = 0;
      matrix = matrix_vec[score_index1];
      out = matrix;
      return;
    }

    //  考察minscore2, score更可信
    //  if (std::abs(min_score1 - min_score2) < scoreThreshold) {
    if (std::abs(min_score1 - min_score2) < 0.1) {
      //  迭代次数相差有点大取min_score2否则取
      if (iteration_vec[score_index1] > iteration_vec[score_index2] && min_score2 < scoreThreshold) {
        // ROS_INFO("Choose Second Score: %f, %d", min_score2, iteration_vec[score_index2]);
        std::cout << "Choose second Score: " << min_score2 << ", " << iteration_vec[score_index2]
                  << std::endl;
        matrix = matrix_vec[score_index2]; // get the min score matrix
        out = matrix;
        code = 0;
        return;
      }
    }

    // 两者score非常接近，再考察iter
    if (std::abs(min_score1 - score_vec[iter_index1]) < 0.1 && score_vec[iter_index1] < scoreThreshold) {
      if (min_iteration1 < iteration_vec[min_score1])  //  考察min_iter
      {
        // ROS_INFO("Choose Best Iteration: %f, %d", score_vec[iter_index1], iteration_vec[iter_index1]);
        std::cout << "Choose Best Iteration: " << score_vec[iter_index1] << ", " << min_iteration1
                  << std::endl;

        matrix = matrix_vec[iter_index1]; // get the min score matrix
        code = 1;
        out = matrix;
        return;
      }
      //   考察min_iter2
      //   if (std::abs(min_score1 - score_vec[iter_index2]) < scoreThreshold) {
      //   if (std::pow((min_score1 - score_vec[iter_index2]), 2) < 1) {
      if (std::abs(min_score1 - score_vec[iter_index2]) < 0.1) {
        // ROS_INFO("Choose Best Iteration: %f, %d", score_vec[iter_index2],
        //         iteration_vec[iter_index2]);
        std::cout << "Choose second Iteration : " << score_vec[iter_index2] << ", " << min_iteration2
                  << std::endl;
        matrix = matrix_vec[iter_index2]; // get the min score matrix
        code = 1;
        out = matrix;
        return;
      }
    }


    // 特殊情况, min_score很小，但iter不是最小的
    //  min score: 0.405817, 14; 3.99814, 8
    //  min Iteration: 16.5084, 6; 6.50399, 6
    if (min_score1 < scoreThreshold && min_iteration1 < iter + 2) {
      //  ROS_INFO("Choose Min Score: %f, %d", min_score1, iteration_vec[score_index1]);
      std::cout << "Choose min Score: " << min_score1 << ", " << iteration_vec[score_index1] << std::endl;
      matrix = matrix_vec[score_index1]; // get the min score matrix
      out = matrix;
      code = 0;
    }
  } else {
    ROS_ERROR("Failed to get score, iteration and transform matrix vec, size %d, %d, %d",
              score_vec.size(), iteration_vec.size(), matrix_vec.size());
  }
}

void GlobalLocalization::RvizCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {

}

void GlobalLocalization::HeadingCallback(const geometry_msgs::QuaternionStamped &msg) {
  auto q = msg.quaternion;
  if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
//    ROS_ERROR("Nan heading, ");
    return;
  }
  gps_heading_time_ = msg.header.stamp;
  gps_heading_ = Eigen::Quaterniond(q.z, 0, 0, q.w); // gps heading to map

  if (has_pos_) {
    gnss_matrix2.block<3, 1>(0, 3) = Eigen::Vector3d(gps_pos_(0), gps_pos_(1), gps_pos_(2));
    gnss_matrix2.block<3, 3>(0, 0) = gps_heading_.toRotationMatrix();
    utils::PublishOdometry(msg.header.stamp, pub_gps2odom2_, "map", "base_link", gnss_matrix2);
    heading_i++;
  }
  has_heading_ = true;
}

void GlobalLocalization::GnssCallback(const sensor_msgs::NavSatFix &msg) {
  gps_all++;
//  std::cout << "heading: " << heading_i << std::endl;
////  std::cout << "NAN_I: " << NAN_I << std::endl;
//  std::cout << "gps: " << gps_i << std::endl;
//  std::cout << "gps_all: " << gps_all << std::endl;
//  std::cout << "gps/heading: " << (heading_i / gps_i) * 1.00 << std::endl;
//
  if (std::isnan(msg.latitude + msg.longitude + msg.altitude)) {
    ROS_ERROR("GPS Position NAN, Waiting for better data...");
    return;
  }
  if (gtools.lla_origin_ == Eigen::Vector3d::Identity()) {
    Eigen::Vector3d lla = gtools.GpsMsg2Eigen(msg);
    gtools.lla_origin_ = lla;
    ROS_ERROR("Please set origin first!!!");
    return;
  }

  Eigen::Vector3d lla = gtools.GpsMsg2Eigen(msg);
  Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
  Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
  gps_pos_ = enu;
  has_pos_ = true;

  gnss_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(gps_pos_(0), gps_pos_(1), gps_pos_(2));

  //  根据运动计算GPS YAW
  double distance = sqrt(pow(enu(1) - enu(1), 2) +
      pow(enu(0) - prev_pos_(0), 2));
  if (distance > 0.2) {
    yaw = atan2(enu(1) - prev_pos_(1),
                enu(0) - prev_pos_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    prev_pos_ = enu;
    gnss_matrix.block<3, 3>(0, 0) = Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z).toRotationMatrix();
    utils::PublishOdometry(msg.header.stamp, pub_gps2odom_, "map", msg.header.frame_id, gnss_matrix);

    _orientation_ready = true;
    gps_i++;
  }

  gnss_buff_mutex_.lock();
  GNSSData gnss_data;
  gnss_data.time = msg.header.stamp;
  gnss_data.latitude = msg.latitude;
  gnss_data.longitude = msg.longitude;
  gnss_data.altitude = msg.altitude;
  gnss_data.status = msg.status.status;
  gnss_data.service = msg.status.service;

  gnss_data.gps_covariance[5] = msg.status.status;
  gnss_data.gps_covariance[0] = msg.position_covariance[0];
  gnss_data.gps_covariance[1] = msg.position_covariance[1];
  gnss_data.gps_covariance[2] = msg.position_covariance[2];
  gnss_data.gps_covariance[6] = msg.position_covariance[3];
  gnss_data.gps_covariance[7] = msg.position_covariance[4];
  gnss_data.gps_covariance[8] = msg.position_covariance[5];
  gnss_data.gps_covariance[12] = msg.position_covariance[6];
  gnss_data.gps_covariance[13] = msg.position_covariance[7];
  gnss_data.gps_covariance[14] = msg.position_covariance[8];

  gnss_data.position = enu;
  gnss_data.orientation = _quat;
  gnss_data.x = enu(0);
  gnss_data.y = enu(1);
  gnss_data.z = enu(2);

  if (has_heading_) {
    gnss_matrix.block<3, 3>(0, 0) = gps_heading_.toRotationMatrix();
    gnss_data.guess_matrix = gnss_matrix;
    gnss_data.service = 2;
  } else if (_orientation_ready) {
    gnss_data.guess_matrix = gnss_matrix;
    gnss_data.service = 1;
  } else {
    gnss_data.service = 0;
  }
  new_gnss_data_.push_back(gnss_data);
  gnss_buff_mutex_.unlock();

  has_heading_ = false;
  _orientation_ready = false;
}

void GlobalLocalization::GuessCallback(const nav_msgs::Odometry::ConstPtr &guess_msgs) {
  if (/*is_lost_*/!pose_success) {
    gnss_buff_mutex_.lock();
    Eigen::Isometry3d e;
    tf::poseMsgToEigen(guess_msgs->pose.pose, e);
    Eigen::Vector4d gps = e.matrix().block(0, 3, 4, 1);

    GNSSData gnss_data;
    gnss_data.time = guess_msgs->header.stamp;
    gnss_data.position = gps.block(0, 0, 3, 1);
    gnss_data.orientation = guess_msgs->pose.pose.orientation;

    gnss_data.x = gnss_data.position(0);
    gnss_data.y = gnss_data.position(1);
    gnss_data.z = gnss_data.position(2);
    //  gnss_data.latitude = msg.latitude;
    //  gnss_data.longitude = msg.longitude;
    //  gnss_data.altitude = msg.altitude;
    //  gnss_data.status = msg.status.status;
    //  gnss_data.service = msg.status.service;

    // 当前的姿态
    gnss_data.guess_matrix.block<3, 3>(0, 0) = Eigen::Quaterniond(gnss_data.orientation.w,
                                                                  gnss_data.orientation.x,
                                                                  gnss_data.orientation.y,
                                                                  gnss_data.orientation.z).toRotationMatrix();
    gnss_data.guess_matrix.block<3, 1>(0, 3) = gnss_data.position;

    gnss_data.gps_covariance = guess_msgs->pose.covariance;
    new_gnss_data_.push_back(gnss_data);

    gnss_buff_mutex_.unlock();
  }

}

void GlobalLocalization::PointsCallback(const sensor_msgs::PointCloud2 &msg) {
  //  已经初始化完成了
  if (/*is_lost_*/ !pose_success) {
    cloud_buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = msg.header.stamp;
    pcl::fromROSMsg(msg, *(cloud_data.cloud_ptr));
    //pcl::fromROSMsg(msg, *lidar_cloud_);
    // scan process
    std::vector<int> indices; //   remove NAN
    pcl::removeNaNFromPointCloud(*(cloud_data.cloud_ptr), *(cloud_data.cloud_ptr), indices);

    if (cloud_data.cloud_ptr->is_dense) {
      utils::Downsample(cloud_data.cloud_ptr, 0.5);
      new_cloud_data_.push_back(cloud_data);
      cloud_buff_mutex_.unlock();
    }
  }
}

void GlobalLocalization::IsLostCallback(const std_msgs::Bool::ConstPtr &lost_msgs) {
  if (!is_lost_ && lost_msgs->data) {
    std::cout << "\n[" << ros::Time::now() << "]*******GOT LOST*******"
              << std::endl;
  }
  is_lost_ = lost_msgs->data;
}

/*
void GlobalLocalization::GnssCallback(const sensor_msgs::NavSatFix &msg) {

  if (std::isnan(msg.latitude + msg.longitude + msg.altitude)) {
    ROS_ERROR("GPS Position NAN, Waiting for better data...");
    return;
  }

  if (msg.status.status == 4 || msg.status.status == 5 || msg.status.status == 1 || msg.status.status == 2) {
    //  第一个的时候设置为起点
    if (gtools.lla_origin_ == Eigen::Vector3d::Identity()) {
      Eigen::Vector3d lla = gtools.GpsMsg2Eigen(msg);
      gtools.lla_origin_ = lla;
    } else {
      Eigen::Vector3d lla = gtools.GpsMsg2Eigen(msg);
      Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
      Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);

      gps_pos_ = enu;
      has_pose = true; //  gps有pose
    }
    //  根据运动计算GPS YAW
    double distance = sqrt(pow(gps_pos_(1) - prev_pos_(1), 2) +
        pow(gps_pos_(0) - prev_pos_(0), 2));
    if (distance > 0.1) {
      yaw = atan2(gps_pos_(1) - prev_pos_(1),
                  gps_pos_(0) - prev_pos_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
      _quat = tf::createQuaternionMsgFromYaw(yaw);
      prev_pos_ = gps_pos_;
    }
    init_p = Eigen::Vector3f(static_cast<float >(gps_pos_(0)), static_cast<float >(gps_pos_(1)),
                             static_cast<float >(gps_pos_(2)));
    init_q = Eigen::Quaternionf(static_cast<float >(_quat.w), static_cast<float >(_quat.x),
                                static_cast<float >(_quat.y), static_cast<float >(_quat.z));

    // pub gps odom
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = init_q.toRotationMatrix();
    m.block<3, 1>(0, 3) = init_p;
    utils::PublishOdometry(msg.header.stamp, gnss_odom_pub, "map", msg.header.frame_id, m.matrix());

    //  发布 map-> base_link
    */
/*transform.setOrigin(tf::Vector3(gps_pos_(0), gps_pos_(1), gps_pos_(2)));
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
    transform.setRotatio n(q);
    broadcaster.sendTransform(
            tf::StampedTransform(transform, msg.header.stamp, "map", "base_link"));*//*

  }

}
*/

/*
void GlobalLocalization::PointsCallback(const sensor_msgs::PointCloud2 &msg) {
  if (!globalmap || (globalmap->size() < 100)) {
    ROS_ERROR("Globalmap has not been received !!!");
    return;
  }
  if (!has_pose) {
    ROS_WARN("Waiting For GNSS Pose ...\n");
    return;
  }
  //  已经初始化完成了
  if (pose_success) {
    ROS_WARN("Pose Already Initialized ...\n");
    utils::PublishPose(msg.header.stamp, init_pub, "map", final_matrix);
    //  publish transform
    geometry_msgs::TransformStamped odom_trans = utils::matrix2transform(msg.header.stamp, final_matrix, "map",
                                                                         "base_link");
    broadcaster.sendTransform(odom_trans);
    return;
  }

  pcl::fromROSMsg(msg, scan);

  // scan process
  std::vector<int> indices; //   remove NAN
  pcl::removeNaNFromPointCloud(scan, scan, indices);
  // crop cloud
  pcl::PointXYZI p;         //   remove too near or far points
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++) {
    p.x = (double) item->x;
    p.y = (double) item->y;
    p.z = (double) item->z;
    p.intensity = (double) item->intensity;

    double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range) {
      filtered_scan.push_back(p);
    }
  }
  //  当前半径100米范围内局部点云地图有效
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filtered_scan));
  tempor_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*globalmap, *tempor_cloud);
  utils::CropCloud(tempor_cloud, circle_radius, init_p);

  auto filtered_transformed_scan = utils::Downsample(transformed_scan_ptr, scan_leaf);
  auto filtered_map = utils::Downsample(tempor_cloud, map_leaf);
  map_points_num = filtered_map->size();
  scan_points_num = filtered_transformed_scan->size();

  if (align_method == 1) {
//    registration->setInputTarget(globalmap); //  设置ndt targetCloud
    registration->setInputTarget(tempor_cloud); //  设置ndt targetCloud
  } else if (align_method == 0) {
    //cpu_ndt.setInputTarget(globalmap);
    cpu_ndt.setInputTarget(tempor_cloud);
  }

  //  发布局部配准的点云地图roi map
  utils::PublishTransformedCloud(msg.header.stamp, roimap_pub, "map", Eigen::Matrix4f::Identity(), *globalmap);
  ros::Time time_1 = ros::Time::now(), time_2 = ros::Time::now(), time_3 = ros::Time::now();

  // 只匹配一次
  if (!first_scan) {
    first_scan = true;
    time_1 = ros::Time::now();
    //  遍历一圈,获取最佳匹配结果
    for (int j = 0; j < testMatrix_vec.size(); ++j) {
      testMatrix_vec[j].block<3, 1>(0, 3) = init_p.matrix();
      if (testMatrix_vec[j] == Eigen::Matrix4f::Identity() || (isnan(testMatrix_vec[j].array())).any()) {
        ROS_ERROR("Testing matrix can not be 0 ...");
        continue;
      }

      if (align_method == 1) {
        AlignPoints(msg.header.stamp, filtered_transformed_scan, testMatrix_vec[j]); // aligned
      } else if (align_method == 0) {
        AlignPointsCpu(msg.header.stamp, filtered_transformed_scan, testMatrix_vec[j]); // aligned
      }

      //                AlignPointsCpu(input.header.stamp, filtered_transformed_scan, filtered_map, testMatrix_vec[j]); // aligned
      //                AlignPoints(input.header.stamp, filtered_transformed_scan, testMatrix_vec[j]); // aligned

      // save ndt scores
      matrix_vec.push_back(ndt_trans);
      score_vec.push_back(fitness_score);
      iteration_vec.push_back(iteration);

      //  如果ndt匹配结果比较好，就直接初始化完成
      if (fitness_score < 1 && iteration < 10 && iteration > 0) {
        ROS_INFO("NDT matching success ...");
        std::cout << "Choose best score and iteration : " << fitness_score << ", " << iteration
                  << std::endl;

        final_scan.clear();
        pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(scan, final_scan);
        final_matrix = ndt_trans;
        utils::PublishPose(msg.header.stamp, init_pub, "map", final_matrix);
        utils::PublishTransformedCloud(msg.header.stamp, final_pub, "map", final_matrix, final_scan);
        //  publish transform
        geometry_msgs::TransformStamped odom_trans = utils::matrix2transform(msg.header.stamp, final_matrix, "map",
                                                                             "base_link");
        broadcaster.sendTransform(odom_trans);

        pose_success = true;
        return;
      }
    }
    time_2 = ros::Time::now();
    CaculateBestYaw();    //  searching best yaw
    time_3 = ros::Time::now();

    std::cout << "1-2: " << (time_2 - time_1) << " s " << "--Total Aligning Time" << std::endl;
    std::cout << "2-3: " << std::setprecision(10) << (time_3 - time_2) << " s " << "--Total Searching Best Yaw"
              << std::endl;

    if (code >= 0) {      //  最小分数的iteration - 最小迭代次数
      pose_success = true;

      final_scan.clear();
      pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(scan, final_scan);
      utils::PublishPose(msg.header.stamp, init_pub, "map", final_matrix);
      utils::PublishTransformedCloud(msg.header.stamp, final_pub, "map", final_matrix, final_scan);
      //  publish transform
      //  geometry_msgs::TransformStamped odom_trans = matrix2transform(msg.header.stamp, final_matrix, "map",
      //                                                    "base_link");
      //  broadcaster.sendTransform(odom_trans);
    } else {
      ROS_ERROR("Waiting for rviz pose!");
      //  等待rviz给定初值，再次进行匹配
      init_rviz = true;
    }
    return;  //  执行结束就退出当前帧
  }


  //  init scan and map
  scan.clear();
  transformed_scan.clear();
  filtered_scan.clear();

}
*/


/*void GlobalLocalization::MapCallback(const std::string &path) {
  if (!load_map) {
    //  ROS_INFO("Loading point cloud map ...");
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile<pcl::PointXYZI>(path, *map_cloud);
    if (map_cloud->size() == 0) {
      ROS_INFO("Map is empty ...");
      return;
    }
    // remove NAN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*map_cloud, *map_cloud, indices);
    //  ROS_INFO("Original map size: %d", map_cloud->size());

    map_cloud = utils::Downsample(map_cloud, map_leaf); //  下采样
    globalmap = map_cloud;
    // registration->setInputTarget(globalmap);

    // publish map
    utils::PublishTransformedCloud(ros::Time::now(), map_pub, "map", Eigen::Matrix4f::Identity(), *globalmap);
    load_map = true;
    //  ROS_INFO("Filtered map size: %d", globalmap->size());
    //  ROS_INFO("Map has been published.");
  }
}

void GlobalLocalization::GlobalMapCallback(const sensor_msgs::PointCloud2 &msg) {
  //  ROS_INFO("Loading point cloud map ...");
  map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(msg, *map_cloud);
  if (map_cloud->size() == 0) {
    ROS_ERROR("Map is empty, please check your map path ...");
    return;
  }
  ROS_INFO("Global Map has been received ...");
  // remove NAN
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_cloud, *map_cloud, indices);
  //  ROS_INFO("Original global map size: %d", map_cloud->size());

  globalmap = utils::Downsample(map_cloud, map_leaf); //  下采样
  globalmap = map_cloud;
  //map_points_num = globalmap->size();

//  if (align_method == 1) {
//    registration->setInputTarget(globalmap); //  设置ndt targetCloud
//  } else if (align_method == 0) {
//    cpu_ndt.setInputTarget(globalmap);
//  }
  //  ROS_INFO("Filtered map size: %d", globalmap->size());
}

void GlobalLocalization::LocalmapCallback(const ros::TimerEvent &event) {
  clock_t start = clock();

  if (has_pose) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointXYZI searchPoint;
    searchPoint.x = gps_pos_(0);
    searchPoint.y = gps_pos_(1);
    searchPoint.z = gps_pos_(2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    float z_min_threshold = lidar_height + trim_low; // 取指定高度区间的点云
    float z_max_threshold = lidar_height + trim_high;

    // 搜索当前位置，x m半斤范围内的点云，id存在pointIdxRadiusSearch中
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      trimmed_cloud->points.reserve(60000);
      for (int i : pointIdxRadiusSearch) {
        if (globalmap->points[i].z > z_min_threshold && map_cloud->points[i].z < z_max_threshold) {
          pcl::PointXYZI cpt;
          cpt.x = map_cloud->points[i].x;
          cpt.y = map_cloud->points[i].y;
          cpt.z = map_cloud->points[i].z;
          cpt.intensity = map_cloud->points[i].intensity;
          trimmed_cloud->points.push_back(cpt);
        }
      }
      trimmed_cloud->width = trimmed_cloud->points.size(); // 电云数量
      trimmed_cloud->height = 1;

//                std::cout << "cropmap.size()\t:" << map_cloud->size() << std::endl << "trimmed_cloud->width:\t"
//                          << trimmed_cloud->width << std::endl;
    }

    ROS_INFO(" local map updated on x:%f, y:%f, z:%f", searchPoint.x, searchPoint.y, searchPoint.z);
    globalmap = trimmed_cloud;
  } else {
    ROS_WARN("Waiting for GNSS to Crop the Map.");
    globalmap = map_cloud;
  }
  registration->setInputTarget(globalmap);
  utils::PublishTransformedCloud(ros::Time::now(), localmap_pub, "map", Eigen::Matrix4f::Identity(), *globalmap);
  std::cout << "Map filtered size: " << globalmap->size() << std::endl;

  clock_t end = clock();
  ROS_INFO("trim time = %f seconds", (double) (end - start) / CLOCKS_PER_SEC);
}*/
}