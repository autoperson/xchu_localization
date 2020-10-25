//
// Created by xchu on 2020/7/3.
//


#include <Eigen/Dense> // eigen库引入需要放在opencv头文件前

#include <iostream>
#include "dirent.h"
#include <ostream>

// corresponding to g2o
struct PointWithTf {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d
            loc;
    Eigen::Quaterniond tf;

    PointWithTf(double x, double y, double z,
                double qx, double qy, double qz, double qw) :
            loc(x, y, z),
            tf(qw, qx, qy, qz) {
    }
};// 读取文件夹内的点云文件，并排序


namespace common_utils {
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

    void LoadG2O(std::string &g2o_file, std::vector<PointWithTf> &g2o_path_) {
        std::ifstream fin(g2o_file);
        if (!fin) {
            //ROS_INFO("[init_location] RUNNING WITHOUT G2O FILE");
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
        //ROS_INFO("[init_location] RUNNING WITH G2O FILE CONTAINING %lu VERTICES",
          //       g2o_path_.size());
    }

    void WriteFile(const std::string &path, std::vector<Eigen::MatrixXd> &sc) {
        std::ofstream fout(path, std::ios::out);
        for (size_t i = 0; i < sc.size(); i++) {
            //fout.precision(8);
            fout << sc[i] << std::endl;
        }
        fout.close();
    }

}

