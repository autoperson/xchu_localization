//
// Created by xchu on 2020/7/3.
//

#include "global_localization/sc_localization.h"

namespace global_localization {
    SCLocalization::SCLocalization() : nh_("~") {
        // 加载全局特征
        ParamInitial();

        // heading_sub = nh_.subscribe(heading_topic, 5, &SCLocalization::HeadingCallback, this);
        gps_sub_ = nh_.subscribe("/novatel718d/pos", 10, &SCLocalization::GnssCallback, this);
        points_sub_ = nh_.subscribe("/velodyne_points", 10, &SCLocalization::PointsCallback, this);

        aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
        gnss_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10, false);

    }

    void SCLocalization::ParamInitial() {

        gpsTools_.lla_origin_ << origin_latitude, origin_longitude, origin_altitude;

        std::string pcd_dir = "/home/xchu/data/yiqingchuangxin_map2.0.0/Raw/pcd_buffer";
        std::string g2o_dir = "/home/xchu/data/yiqingchuangxin_map2.0.0/Raw/pose_graph.g2o";

        //std::cout << "pcd_dir:" << pcd_dir << std::endl;
        std::vector<std::string> pcds_vec;
        getAllFiles(pcd_dir, pcds_vec);

        // load g2o files
        LoadG2O(g2o_dir, g2o_pose_vec_);
        std::cout << "g2o size: " << pcds_vec.size() << "," << g2o_pose_vec_.size() << std::endl;
        //for (int j = 0; j < g2o_pose_vec_.size(); ++j) {

        //}

        //-------------------------------1. 地图预处理------------------------------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_;
        SC2::SCManager scManager1;

        for (int i = 0; i < pcds_vec.size(); ++i) {
            frame_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcds_vec[i], *frame_cloud_) == -1) {
                PCL_ERROR("Couldn't read map file\n");
            }

            scManager1.clear();

            Eigen::MatrixXd sc = scManager1.makeScancontext(*frame_cloud_); // v1  20*60
            Eigen::MatrixXd ringkey = scManager1.makeRingkeyFromScancontext(sc); // ring key (ring索引)  20*1
            Eigen::MatrixXd sectorkey = scManager1.makeSectorkeyFromScancontext(sc); // sector key (区块索引) 60*1
            std::vector<float> polarcontext_invkey_vec = SC2::eig2stdvec(ringkey);  // vector形式的ring key

            polarcontexts_vec_.push_back(sc);
            polarcontext_invkeys_vec_.push_back(ringkey);
            polarcontext_vkeys_vec_.push_back(ringkey);
            polarcontext_invkeys_mat_vec_.push_back(polarcontext_invkey_vec);
        }

        std::cout << "size : " << polarcontexts_vec_.size() << "," << polarcontext_invkeys_vec_.size()
                  << ", " << polarcontext_vkeys_vec_.size() << ","
                  << polarcontext_invkeys_mat_vec_.size() << std::endl;

        //-------------------------------3. 构造全局搜索树------------------------------------------
        if (polarcontext_invkeys_mat_vec_.size() < scManager.NUM_EXCLUDE_RECENT + 1) {
            ROS_ERROR("init failed!!!");
        }

        std::cout << "Tree construction begin..." << std::endl;
        scManager.clear();
        TicToc t_tree_construction;
        polarcontext_invkeys_to_search_.clear();
        //  容器赋值，将所有关键帧的key丢进去
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_vec_.begin(),
                                               polarcontext_invkeys_mat_vec_.end());
        polarcontext_tree_.reset();
        polarcontext_tree_ = std::make_unique<InvKeyTree>(scManager.PC_NUM_RING /* dim */,
                                                          polarcontext_invkeys_to_search_,
                                                          10 /* max leaf */ );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
        std::cout << "Tree construction over, polarcontext_invkeys_to_search_ size is "
                  << polarcontext_invkeys_to_search_.size() << std::endl;

    }


    void SCLocalization::PointsCallback(const sensor_msgs::PointCloud2 &msg) {

        frame_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filter_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(msg, *frame_cloud_);

        if (frame_cloud_->empty()) {
            ROS_ERROR("empty point cloud..");
            return;
        }
        // scan process
        std::vector<int> indices; //   remove NAN
        pcl::removeNaNFromPointCloud(*frame_cloud_, *frame_cloud_, indices);
        pcl::PointXYZI p;         //   remove too near or far points
        for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = frame_cloud_->begin();
             item != frame_cloud_->end(); item++) {
            p.x = (double) item->x;
            p.y = (double) item->y;
            p.z = (double) item->z;
            p.intensity = (double) item->intensity;

            double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
            if (2 < r && r < 100) {
                filter_cloud_->push_back(p);
            }
        }

        // 当前帧提取sc 和 key, 去检测
        scManager.clear();
        Eigen::MatrixXd cur_desc = scManager.makeScancontext(*filter_cloud_); // v1  20*60
        Eigen::MatrixXd cur_ringkey = scManager.makeRingkeyFromScancontext(cur_desc); // ring key (ring索引)  20*1
        // Eigen::MatrixXd cur_sectorkey = scManager.makeSectorkeyFromScancontext(cur_sc); // sector key (区块索引) 60*1
        std::vector<float> cur_polarcontext_invkey_vec = SC2::eig2stdvec(cur_ringkey);  // vector形式的ring key

        std::cout << "current ring key size: " << cur_polarcontext_invkey_vec.size() << std::endl;

        int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
        double min_dist = 10000000; // init with somthing large
        int nn_align = 0;  //  最后返回的帧score
        int nn_idx = 0; //  最后返回的帧id

        // knn search
        std::vector<size_t> candidate_indexes(scManager.NUM_CANDIDATES_FROM_TREE); // 只需要10个候选帧
        std::vector<float> out_dists_sqr(scManager.NUM_CANDIDATES_FROM_TREE);

        TicToc t_tree_search;
        nanoflann::KNNResultSet<float> knnsearch_result(scManager.NUM_CANDIDATES_FROM_TREE);
        knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
        // 寻找和当前的key接近的10帧候选
        polarcontext_tree_->index->findNeighbors(knnsearch_result, &cur_polarcontext_invkey_vec[0],
                                                 nanoflann::SearchParams(10));
        t_tree_search.toc("Tree search");

        TicToc t_calc_dist;
        for (int candidate_iter_idx = 0;
             candidate_iter_idx < scManager.NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++) {

            //  寻找当前sc和候选sc的score, 只比较候选的10帧
            MatrixXd polarcontext_candidate = polarcontexts_vec_[candidate_indexes[candidate_iter_idx]];
            std::pair<double, int> sc_dist_result = scManager.distanceBtnScanContext(cur_desc, polarcontext_candidate);

            double candidate_dist = sc_dist_result.first;
            int candidate_align = sc_dist_result.second;

            // 在候选的10帧里面找到score最小的，返回其index
            if (candidate_dist < min_dist) {
                min_dist = candidate_dist;
                nn_align = candidate_align;

                nn_idx = candidate_indexes[candidate_iter_idx];
            }
        }
        t_calc_dist.toc("Distance calc");

        /*
         * loop threshold check
         */
        if (min_dist < scManager.SC_DIST_THRES) {
            loop_id = nn_idx;
            // std::cout.precision(3);
            cout << "[Loop found] Nearest distance: " << min_dist << " nn_idx " << nn_idx << "." << endl;
            cout << "[Loop found] yaw diff: " << nn_align * scManager.PC_UNIT_SECTORANGLE << " deg." << endl;

            std::cout << "[Loop Found] pose:" << g2o_pose_vec_[nn_idx].loc << std::endl;

            Eigen::Quaterniond quaterniond_ = g2o_pose_vec_[nn_idx].tf;
            Eigen::Vector3d pose_ = g2o_pose_vec_[nn_idx].loc;

            Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
            init_guess.block<3, 3>(0, 0) = quaterniond_.toRotationMatrix();
            init_guess.block<3, 1>(0, 3) = pose_;

            //  publish cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*frame_cloud_, *aligned_cloud, init_guess);
            sensor_msgs::PointCloud2 fScan;
            pcl::toROSMsg(*aligned_cloud, fScan);

            fScan.header.stamp = msg.header.stamp;
            fScan.header.frame_id = "map";
            aligned_pub_.publish(fScan);
        } else {
            std::cout.precision(3);
            cout << "[Not loop] Nearest distance: " << min_dist << " nn_idx " << nn_idx << "." << endl;
            cout << "[Not loop] yaw diff: " << nn_align * scManager.PC_UNIT_SECTORANGLE << " deg." << endl;
        }

        // To do: return also nn_align (i.e., yaw diff)
        float yaw_diff_rad = SC2::deg2rad(nn_align * scManager.PC_UNIT_SECTORANGLE);
        std::pair<int, float> result{loop_id, yaw_diff_rad};

    }

    void SCLocalization::GnssCallback(const sensor_msgs::NavSatFix &msg) {

        if (std::isnan(msg.latitude + msg.longitude + msg.altitude)) {
            ROS_ERROR("GPS Position NAN, Waiting for better data...");
            return;
        }

        if (msg.status.status == 4 || msg.status.status == 5 || msg.status.status == 1 || msg.status.status == 2) {
            //  第一个的时候设置为起点
            if (gpsTools_.lla_origin_ == Eigen::Vector3d::Identity()) {
                Eigen::Vector3d lla = gpsTools_.GpsMsg2Eigen(msg);
                gpsTools_.lla_origin_ = lla;
            } else {
                Eigen::Vector3d lla = gpsTools_.GpsMsg2Eigen(msg);
                Eigen::Vector3d ecef = gpsTools_.LLA2ECEF(lla);
                Eigen::Vector3d enu = gpsTools_.ECEF2ENU(ecef);

                gps_pose_ = enu;
                // has_pose = true; //  gps有pose
            }
            //  根据运动计算GPS YAW
            double distance = sqrt(pow(gps_pose_(1) - prev_pose_(1), 2) +
                                   pow(gps_pose_(0) - prev_pose_(0), 2));
            if (distance > 0.1) {
                yaw = atan2(gps_pose_(1) - prev_pose_(1),
                            gps_pose_(0) - prev_pose_(0)); // 返回值是此点与远点连线与x轴正方向的夹角
                _quat = tf::createQuaternionMsgFromYaw(yaw);
                prev_pose_ = gps_pose_;
            }
            init_p = Eigen::Vector3f(static_cast<float >(gps_pose_(0)), static_cast<float >(gps_pose_(1)),
                                     static_cast<float >(gps_pose_(2)));
            init_q = Eigen::Quaternionf(static_cast<float >(_quat.w), static_cast<float >(_quat.x),
                                        static_cast<float >(_quat.y), static_cast<float >(_quat.z));

            // pub gps odom
            Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
            m.block<3, 3>(0, 0) = init_q.toRotationMatrix();
            m.block<3, 1>(0, 3) = init_p;
            PublishOdometry(msg.header.stamp, gnss_odom_pub_, "map", msg.header.frame_id, m.matrix());

            //  发布 map-> base_link
            /*transform.setOrigin(tf::Vector3(gps_pos_(0), gps_pos_(1), gps_pos_(2)));
            tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
            transform.setRotation(q);
            broadcaster.sendTransform(
                    tf::StampedTransform(transform, msg.header.stamp, "map", "base_link"));*/
        }


    }

    void SCLocalization::PublishOdometry(const ros::Time &time,
                                         const ros::Publisher &topic_pub,
                                         const std::string &base_frame_id,
                                         const std::string &child_frame_id,
                                         const Eigen::Matrix4f &transform_matrix) {
        nav_msgs::Odometry odometry_;
        odometry_.header.stamp = time;
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;

        //set the position
        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        topic_pub.publish(odometry_);
    }

    void SCLocalization::getAllFiles(std::string path, std::vector<std::string> &files) {
        if (path[path.length() - 1] != '/')
            path = path + "/";
        DIR *dir;
        struct dirent *ptr;
        char base[1000];
        if ((dir = opendir(path.c_str())) == NULL) {
            perror("Open dir error...");
            // std::cout << "Check: " << path << std::endl;
            exit(1);
        }

        while ((ptr = readdir(dir)) != NULL) {
            if (ptr->d_type == 8) // 文件
            {
                std::string name = ptr->d_name;
                int length = name.length();
                if (name.substr(length - 3, length - 1) == "pcd" || name.substr(length - 3, length - 1) == "PCD") {
                    std::cout << path + name << std::endl;
                    files.push_back(path + name);
                }
            }
        }
        closedir(dir);
        // std::sort(files.begin(), files.end());
        return;
    }

    void SCLocalization::LoadG2O(const std::string &g2o_file, std::vector<PointWithTf> &g2o_path_) {
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
        ROS_INFO("[init_location] RUNNING WITH G2O FILE CONTAINING %lu VERTICES",
                 g2o_path_.size());
    }

    void SCLocalization::WriteFile(const std::string &path, std::vector<Eigen::MatrixXd> &sc) {
        std::ofstream fout(path, std::ios::out);
        for (size_t i = 0; i < sc.size(); i++) {
            //fout.precision(8);
            fout << sc[i] << std::endl;
        }
        fout.close();
    }

}