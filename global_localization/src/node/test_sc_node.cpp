//
// Created by xchu on 2020/7/1.
//

//#include "scan_context_node.h"
#include "ros/ros.h"
#include "scan_context/Scancontext.h"
#include <iostream>

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

void getAllFiles(std::string path, std::vector<std::string> &files) {
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

void LoadG2O(const std::string &g2o_file, std::vector<PointWithTf> &g2o_path_) {
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

void WriteFile(const std::string &path, std::vector<Eigen::MatrixXd> &sc) {
    std::ofstream fout(path, std::ios::out);
    for (size_t i = 0; i < sc.size(); i++) {
        //fout.precision(8);
        fout << sc[i] << std::endl;
    }
    fout.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "points_prefilter_node");
    ROS_INFO("\033[1;32m---->\033[0m Scan Context Search Started.");

    // // loop detector

    // 对每一帧点云进行特征提取，然后保存到文件里面，内存里面的提供实时的匹配
    // 重定位的时候根据两轮筛选
    std::string pcd_dir = "/home/xchu/data/yiqingchuangxin_map2.0.0/Raw/pcd_buffer";
    // add slash if it doesn't exist
    if (pcd_dir.back() != '/') {
        pcd_dir.append("/");
    }
    std::cout << "pcd_dir:" << pcd_dir << std::endl;
    std::vector<std::string> pcds_vec;
    getAllFiles(pcd_dir, pcds_vec);

    // load g2o files
    std::vector<PointWithTf> g2o_pose_vec_;
    std::string g2o_dir = "/home/xchu/data/yiqingchuangxin_map2.0.0/Raw/pose_graph.g2o";
    LoadG2O(g2o_dir, g2o_pose_vec_);

    std::cout << "g2o size: " << pcds_vec.size() << "," << g2o_pose_vec_.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_;


    //  map里面的sc
    std::vector<Eigen::MatrixXd> polarcontexts_vec_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_vec_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_vec_;
    KeyMat polarcontext_invkeys_mat_vec_;

    //-------------------------------1. 地图预处理------------------------------------------
    for (int i = 0; i < pcds_vec.size(); ++i) {

        frame_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcds_vec[i], *frame_cloud_) == -1) {
            PCL_ERROR("Couldn't read map file\n");
            return -1;
        }
        SC2::SCManager scManager;
        scManager.clear();

        /* scManager.makeAndSaveScancontextAndKeys(*frame_cloud_);
         std::vector<Eigen::MatrixXd> sc = scManager.polarcontexts_;
         std::vector<Eigen::MatrixXd> ring_key = scManager.polarcontext_invkeys_;
         std::vector<Eigen::MatrixXd> sector_key = scManager.polarcontext_vkeys_;
         KeyMat mat = scManager.polarcontext_invkeys_mat_; // ring key to vector (keymap)*/

        Eigen::MatrixXd sc = scManager.makeScancontext(*frame_cloud_); // v1  20*60
        Eigen::MatrixXd ringkey = scManager.makeRingkeyFromScancontext(sc); // ring key (ring索引)  20*1
        Eigen::MatrixXd sectorkey = scManager.makeSectorkeyFromScancontext(sc); // sector key (区块索引) 60*1
        std::vector<float> polarcontext_invkey_vec = SC2::eig2stdvec(ringkey);  // vector形式的ring key

//        polarcontexts_.push_back(sc);
//        polarcontext_invkeys_.push_back(ringkey);
//        polarcontext_vkeys_.push_back(sectorkey);
//        polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);

        polarcontexts_vec_.push_back(sc);
        polarcontext_invkeys_vec_.push_back(ringkey);
        polarcontext_vkeys_vec_.push_back(ringkey);
        polarcontext_invkeys_mat_vec_.push_back(polarcontext_invkey_vec);
        // 写文件
//        std::string sc_path = pcd_dir + "sc/" + std::to_string(i) + "_sc.txt";
//        std::string ring_path = pcd_dir + "sc/" + std::to_string(i) + "_ring.txt";
//        std::string sec_path = pcd_dir + "sc/" + std::to_string(i) + "_sector.txt";
//
//        std::cout << "write sc describor. " << i << std::endl;
//        WriteFile(sc_path, sc);
//        WriteFile(ring_path, ring_key);
//        WriteFile(sec_path, sector_key);
        //  将其分别保存到文件
    }

    std::cout << "size : " << polarcontexts_vec_.size() << "," << polarcontext_invkeys_vec_.size()
              << ", " << polarcontext_vkeys_vec_.size() << ","
              << polarcontext_invkeys_mat_vec_.size() << std::endl;

    SC2::SCManager scManager;
    //-------------------------------2. 构造全局搜索树------------------------------------------
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    TicToc t_tree_construction;
    polarcontext_invkeys_to_search_.clear();
    polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_vec_.begin(),
                                           polarcontext_invkeys_mat_vec_.end() - scManager.NUM_EXCLUDE_RECENT);

    polarcontext_tree_.reset();
    polarcontext_tree_ = std::make_unique<InvKeyTree>(scManager.PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_,
                                                      10 /* max leaf */ );
    // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
    t_tree_construction.toc("Tree construction");


    //-------------------------------3. 提取当前帧特征------------------------------------------
    // 当前帧提取sc 和 key, 去检测
    scManager.clear();
    Eigen::MatrixXd cur_sc = scManager.makeScancontext(*frame_cloud_); // v1  20*60
    Eigen::MatrixXd cur_ringkey = scManager.makeRingkeyFromScancontext(cur_sc); // ring key (ring索引)  20*1
    Eigen::MatrixXd cur_sectorkey = scManager.makeSectorkeyFromScancontext(cur_sc); // sector key (区块索引) 60*1
    std::vector<float> cur_polarcontext_invkey_vec = SC2::eig2stdvec(cur_ringkey);  // vector形式的ring key

    // auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query) 容器末尾元素
    // auto curr_desc = polarcontexts_.back(); // current observation (query)

    /*
    *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
    */
    int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t> candidate_indexes(scManager.NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr(scManager.NUM_CANDIDATES_FROM_TREE);

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result(scManager.NUM_CANDIDATES_FROM_TREE);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    // 寻找和当前的key接近的10帧候选
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &cur_polarcontext_invkey_vec[0] /* query */,
                                             nanoflann::SearchParams(10));
    t_tree_search.toc("Tree search");

    TicToc t_calc_dist;
    for (int candidate_iter_idx = 0; candidate_iter_idx < scManager.NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++) {
        MatrixXd polarcontext_candidate = polarcontexts_vec_[candidate_indexes[candidate_iter_idx]];
        std::pair<double, int> sc_dist_result = scManager.distanceBtnScanContext(cur_sc, polarcontext_candidate);

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

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
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_vec_.size() - 1 << " and "
             << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * scManager.PC_UNIT_SECTORANGLE << " deg." << endl;
    } else {
        std::cout.precision(3);
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_vec_.size() - 1 << " and "
             << nn_idx
             << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * scManager.PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad =  SC2::deg2rad(nn_align * scManager.PC_UNIT_SECTORANGLE);
    std::pair<int, float> result{loop_id, yaw_diff_rad};




    // 根据当前帧和地图帧，进行检测

    /*pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudRawDS, *thisRawCloudKeyFrame);
    scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);


    //  这里检测回环
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    SCclosestHistoryFrameID = detectResult.first;
    yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)*/

    return 0;
}