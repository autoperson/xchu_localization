
// 引入外部头文件
#include <utils/registrations.hpp>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <iostream>

namespace utils {
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>>
SelectRegistrationMethod(ros::NodeHandle &pnh) {
  using PointT = pcl::PointXYZI;
  // select a registration method (ICP, GICP, NDT)
  std::string registration_method = pnh.param<std::string>("registration_method", "NDT_OMP");
  if (registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(
        new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(pnh.param<double>("transformation_epsilon", 0.01));
    icp->setMaximumIterations(pnh.param<int>("maximum_iterations", 64));
    icp->setUseReciprocalCorrespondences(pnh.param<bool>("use_reciprocal_correspondences", false));
    return icp;
  } else if (registration_method.find("GICP") != std::string::npos) {
    if (registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(
          new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(pnh.param<double>("transformation_epsilon", 0.01));
      gicp->setMaximumIterations(pnh.param<int>("maximum_iterations", 64));
      gicp->setUseReciprocalCorrespondences(pnh.param<bool>("use_reciprocal_correspondences", false));
      gicp->setCorrespondenceRandomness(pnh.param<int>("gicp_correspondence_randomness", 20));
      gicp->setMaximumOptimizerIterations(pnh.param<int>("gicp_max_optimizer_iterations", 20));
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(
          new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(pnh.param<double>("transformation_epsilon", 0.01));
      gicp->setMaximumIterations(pnh.param<int>("maximum_iterations", 64));
      gicp->setUseReciprocalCorrespondences(pnh.param<bool>("use_reciprocal_correspondences", false));
      gicp->setCorrespondenceRandomness(pnh.param<int>("gicp_correspondence_randomness", 20));
      gicp->setMaximumOptimizerIterations(pnh.param<int>("gicp_max_optimizer_iterations", 20));
      return gicp;
    }
  } else {
    if (registration_method.find("NDT") == std::string::npos) {
      std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
      std::cerr << "       : use NDT" << std::endl;
    }

    double ndt_resolution = pnh.param<double>("ndt_resolution", 0.5);
    if (registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: NDT " << ndt_resolution << std::endl;
      boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(
          new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(pnh.param<double>("transformation_epsilon", 0.01));
      ndt->setMaximumIterations(pnh.param<int>("maximum_iterations", 64));
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      int num_threads = pnh.param<int>("ndt_num_threads", 0);
      std::string nn_search_method = pnh.param<std::string>("ndt_nn_search_method", "DIRECT7");
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " ("
                << num_threads << " threads)" << std::endl;
      boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(
          new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if (num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon(pnh.param<double>("transformation_epsilon", 0.01));
      ndt->setMaximumIterations(pnh.param<int>("maximum_iterations", 64));
      ndt->setResolution(ndt_resolution);
      if (nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if (nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
    }
  }
  return nullptr;
}

/* bool PoseInitController::align_points_teaser(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan,
                                              const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &map,
                                              const Eigen::Matrix4f &guess_matrix) {

     // Convert the point cloud to Eigen
     int N = scan->size();
     int M = map->size();
     Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, N), target(3, M);
     for (size_t i = 0; i < N; ++i) {
         source.col(i) << scan->points[i].x, scan->points[i].y, scan->points[i].z;
     }
     for (size_t i = 0; i < M; ++i) {
         target.col(i) << map->points[i].x, map->points[i].y, map->points[i].z;
     }

     // Homogeneous coordinates
     Eigen::Matrix<double, 4, Eigen::Dynamic> src_h, tar_h;
     src_h.resize(4, source.cols());
     src_h.topRows(3) = source;
     src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

     tar_h.resize(4, target.cols());
     tar_h.topRows(3) = target;
     tar_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(M);

     // Apply transformation
     Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
     matrix.block<3, 1>(0, 3) = guess_matrix.block<3, 1>(0, 3).cast<double>();

//        Eigen::Matrix4d guess_matrixd = guess_matrix.cast<double>();
//        Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = guess_matrixd * src_h;
     Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = matrix * src_h;
     Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);
     // Add some noise & outliers
     //        addNoiseAndOutliers(tgt);

     // Run TEASER++ registration
     // Prepare solver parameters
     teaser::RobustRegistrationSolver::Params params;
     params.noise_bound = NOISE_BOUND;
     params.cbar2 = 1;
     params.estimate_scaling = false;
     params.rotation_max_iterations = 100;
     params.rotation_gnc_factor = 1.4;
     params.rotation_estimation_algorithm =
             teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
     params.rotation_cost_threshold = 0.005;

     // Solve with TEASER++
     teaser::RobustRegistrationSolver solver(params);
     std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
     solver.solve(source, tgt);
     std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

     auto solution = solver.getSolution();

     // publish the transformed PointCloud
     pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
     Eigen::Matrix4f final_guess = Eigen::Matrix4f::Identity();
     final_guess.block<3, 3>(0, 0) = solution.rotation.matrix().cast<float>();
     final_guess.block<3, 1>(0, 3) = solution.translation.matrix().cast<float>();
     pcl::transformPointCloud(*scan, *aligned_cloud, final_guess);

     if (aligned_pub.getNumSubscribers()) {
         aligned_cloud->header.frame_id = "map";
         aligned_cloud->header.stamp = scan->header.stamp;

         sensor_msgs::PointCloud2 aligned;
         pcl::toROSMsg(*aligned_cloud, aligned);
         aligned_pub.publish(aligned);
     }
     //   原始点云->配准之后的点云
     //        发布初始化配准之后的点云
     sensor_msgs::PointCloud2 fScan;
     pcl::toROSMsg(*aligned_cloud, fScan);
     fScan.header.frame_id = "map";
     fscan_pub.publish(fScan);

     // Compare results
     std::cout << "=====================================" << std::endl;
     std::cout << "          TEASER++ Results           " << std::endl;
     //        std::cout << "=====================================" << std::endl;
     //        std::cout << "Expected rotation: " << std::endl;
     //        std::cout << guess_matrix.topLeftCorner(3, 3) << std::endl;
     //        std::cout << "Number of correspondences: " << N << std::endl;
//        std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
     //        std::cout << "Estimated rotation: " << std::endl;
     //        std::cout << solution.rotation << std::endl;
//        std::cout << "Error (deg): " << getAngularError(guess_matrixd.topLeftCorner(3, 3), solution.rotation)
     std::cout << "Error (deg): " << getAngularError(matrix.topLeftCorner(3, 3), solution.rotation)
               << std::endl;
     //        std::cout << "Expected translation: " << std::endl;
     //        std::cout << guess_matrix.topRightCorner(3, 1) << std::endl;
     //        std::cout << "Estimated translation: " << std::endl;
     //        std::cout << solution.translation << std::endl;
     std::cout << "Error (m): " << (matrix.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
//        std::cout << "Error (m): " << (guess_matrixd.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
     std::cout << "Time taken (s): "
               << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                  1000000.0
               << std::endl;
     std::cout << "=====================================" << std::endl;

     return false;
 }*/


}
