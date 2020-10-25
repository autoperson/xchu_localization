// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "roadside_writer.h"
#include <opendrive_common.h>
#include <pcl/io/pcd_io.h>           //PCD读写类相关的头文件
#include <pcl/point_types.h>      //PCL中支持的点类型的头文件

namespace opendrive_generator {
    RoadsideWriter::RoadsideWriter() {
        factory_roadside_builder_.SetRoadsideBuilder(&(roadside::UnionRoadsideBuilder::GetInstance()));
    }

    void RoadsideWriter::Write(const std::string &file_path, const hdmap::EntireMap &entire_map) {
        factory_roadside_builder_.Build(entire_map, &polygons_);
        WriteFile(file_path, polygons_);
    }

    void RoadsideWriter::WriteFile(const std::string &file_path,
                                   const std::vector <opendrive_common::GeometryPolygon> &polygons) {
        opendrive_common::FileOperator::Mkdir(file_path);
        opendrive_common::FileOperator::DeleleFileBySuffix(file_path, "pcd");
        std::cout << "polygons.size()=" << polygons.size() << std::endl;
        for (int i = 0; i < polygons.size(); i++) {
            std::string filename = file_path + "/" + std::to_string(i) + ".pcd";
            pcl::PointCloud <pcl::PointXYZ> cloud;

            // 创建点云  并设置适当的参数（width height is_dense）
            cloud.width = polygons[i].vertexs.size();
            cloud.height = 1;
            cloud.is_dense = false;  //不是稠密型的
            cloud.points.resize(cloud.width * cloud.height);  //点云总数大小
            //填充PointCloud点云对象
            for (int j = 0; j < polygons[i].vertexs.size(); j++) {
                cloud.points[j].x = polygons[i].vertexs[j](0);
                cloud.points[j].y = polygons[i].vertexs[j](1);
                cloud.points[j].z = 0;
            }
            //把PointCloud对象数据存储在 test_pcd.pcd文件中
            pcl::io::savePCDFileASCII(filename, cloud);

        }
    }
};  // namespace opendrive_generator
