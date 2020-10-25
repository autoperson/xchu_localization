// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include <string>
#include "file_writer.h"
#include <cstdio>
#include <algorithm>

namespace opendrive_generator {
    FileWriter::FileWriter() {
        topomap_writer_ = &(TopomapWriter::GetInstance());
        routemap_writer_ = &(RoutemapWriter::GetInstance());
        lane_info_writer_ = &(LaneInfoWriter::GetInstance());
        roadside_writer_ = &(RoadsideWriter::GetInstance());
    }

    void FileWriter::WriteFiles(const hdmap::EntireMap &entire_map,
                                const std::string &topomap_path,
                                const std::string &routemap_path,
                                const std::string &lane_info_path,
                                const std::string &roadside_path) {
        topomap_writer_->Write(topomap_path, entire_map);
        routemap_writer_->Write(routemap_path, entire_map);
        lane_info_writer_->Write(lane_info_path, entire_map);
        roadside_writer_->Write(roadside_path, entire_map);
    }

};  // namespace FileWriterNS

/* backup
void FileWriter::WriteNodes(const FullMap::MapManager &map_manager, const
std::string &nodes_path)
{
  node_write_list_.clear();
  for (auto &lane:map_manager.lane_list_.lanes)
  {

    if (lane.type != Lane::DRIVING) continue;

    if (lane.direction == Lane::FORWARD || lane.direction == Lane::UNDIRECTED)
    {
      node_write_list_.emplace_back(NodeWriteType(
          lane.full_id.road_id, lane.full_id.lane_section_id,
          lane.full_id.lane_id, 0, lane.length));
    }
    if (lane.direction == Lane::BACKWARD || lane.direction == Lane::UNDIRECTED)
    {
      node_write_list_.emplace_back(NodeWriteType(
          lane.full_id.road_id, lane.full_id.lane_section_id,
          lane.full_id.lane_id, 1, lane.length));
    }
  }

  std::sort(node_write_list_.begin(), node_write_list_.end());
  FILE *fpt = fopen(nodes_path.c_str(), "w");
  for (const auto &node_write : node_write_list_) {
    fprintf(fpt, "%s %d %d %d %f\n",
            node_write.road_id.c_str(),
            node_write.laneSection_id,
            node_write.lane_id,
            node_write.direction,
            node_write.length);
  }
  fclose(fpt);
}

void FileWriter::WriteEdges(const FullMap::MapManager &map_manager, const
std::string &edges_path)
{
  edge_write_list_.clear();
  for (auto &lane:map_manager.lane_list_.lanes)
  {

    if (lane.type != Lane::DRIVING) continue;
    if (lane.direction == Lane::FORWARD || lane.direction == Lane::UNDIRECTED)
    {
      for (auto &lane_edge:lane.next_lane_edges)
      {
        const auto &
going_lane=map_manager.lane_list_.lanes[lane_edge.second.go_id];     if
(lane_edge.second.going_connection_type==LaneEdge::START &&
(going_lane.direction == Lane::FORWARD || going_lane.direction ==
Lane::UNDIRECTED))
        {
          edge_write_list_.emplace_back(
            EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,
                          going_lane.full_id.road_id,
going_lane.full_id.lane_section_id,going_lane.full_id.lane_id, 0,     0));
        }
        else
        if (lane_edge.second.going_connection_type==LaneEdge::END &&
(going_lane.direction == Lane::BACKWARD || going_lane.direction ==
Lane::UNDIRECTED))
        {
          edge_write_list_.emplace_back(
            EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,
                          going_lane.full_id.road_id,
going_lane.full_id.lane_section_id,going_lane.full_id.lane_id, 1,     0));
        }

      }
      if (lane.can_go_to_left_lane)
      {
        const auto & left_lane=map_manager.lane_list_.lanes[lane.left_lane_id];
        if (left_lane.type != Lane::DRIVING)
        {
          ROS_ERROR("[WriteEdges]: lanes struct error 1.
lane.full_id=(%s,%d,%d),
left_lane.full_id=(%s,%d,%d)",lane.full_id.road_id.c_str(),
lane.full_id.lane_section_id,lane.full_id.lane_id,left_lane.full_id.road_id.c_str(),
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id);     return;
        }
        if (left_lane.direction == Lane::FORWARD || left_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,     left_lane.full_id.road_id,
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id, 0,     1));
        }
        if (left_lane.direction == Lane::BACKWARD || left_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,     left_lane.full_id.road_id,
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id, 1,     3));
        }
      }
      if (lane.can_go_to_right_lane)
      {
        const auto &
right_lane=map_manager.lane_list_.lanes[lane.right_lane_id];     if (right_lane.type
!= Lane::DRIVING)
        {
          ROS_ERROR("[WriteEdges]: lanes struct error 2.
lane.full_id=(%s,%d,%d),
left_lane.full_id=(%s,%d,%d)",lane.full_id.road_id.c_str(),
lane.full_id.lane_section_id,lane.full_id.lane_id,right_lane.full_id.road_id.c_str(),
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id);     return;
        }
        if (right_lane.direction == Lane::FORWARD || right_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,
                            right_lane.full_id.road_id,
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id, 0,     2));
        }
        if (right_lane.direction == Lane::BACKWARD || right_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 0,
                            right_lane.full_id.road_id,
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id, 1,     4));
        }
      }
    }

    if (lane.direction == Lane::BACKWARD || lane.direction == Lane::UNDIRECTED)
    {
      for (auto &lane_edge:lane.next_lane_edges)
      {

        const auto &
going_lane=map_manager.lane_list_.lanes[lane_edge.second.go_id];     if
(lane_edge.second.going_connection_type==LaneEdge::START &&
(going_lane.direction == Lane::FORWARD || going_lane.direction ==
Lane::UNDIRECTED))
        {
          edge_write_list_.emplace_back(
            EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,
                          going_lane.full_id.road_id,
going_lane.full_id.lane_section_id,going_lane.full_id.lane_id, 0,     0));
        }
        else
        if (lane_edge.second.going_connection_type==LaneEdge::END &&
(going_lane.direction == Lane::BACKWARD || going_lane.direction ==
Lane::UNDIRECTED))
        {
          edge_write_list_.emplace_back(
            EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,
                          going_lane.full_id.road_id,
going_lane.full_id.lane_section_id,going_lane.full_id.lane_id, 1,     0));
        }

      }
      if (lane.can_go_to_left_lane)
      {
        const auto & left_lane=map_manager.lane_list_.lanes[lane.left_lane_id];
        if (left_lane.type != Lane::DRIVING)
        {
          ROS_ERROR("[WriteEdges]: lanes struct error 3.
lane.full_id=(%s,%d,%d),
left_lane.full_id=(%s,%d,%d)",lane.full_id.road_id.c_str(),
lane.full_id.lane_section_id,lane.full_id.lane_id,left_lane.full_id.road_id.c_str(),
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id);     return;
        }
        if (left_lane.direction == Lane::FORWARD || left_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,     left_lane.full_id.road_id,
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id, 0,     4));
        }
        if (left_lane.direction == Lane::BACKWARD || left_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,     left_lane.full_id.road_id,
left_lane.full_id.lane_section_id,left_lane.full_id.lane_id, 1,     2));
        }
      }
      if (lane.can_go_to_right_lane)
      {
        const auto &
right_lane=map_manager.lane_list_.lanes[lane.right_lane_id];     if (right_lane.type
!= Lane::DRIVING)
        {
          ROS_ERROR("[WriteEdges]: lanes struct error 4.
lane.full_id=(%s,%d,%d),
left_lane.full_id=(%s,%d,%d)",lane.full_id.road_id.c_str(),
lane.full_id.lane_section_id,lane.full_id.lane_id,right_lane.full_id.road_id.c_str(),
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id);     return;
        }
        if (right_lane.direction == Lane::FORWARD || right_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,
                            right_lane.full_id.road_id,
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id, 0,     3));
        }
        if (right_lane.direction == Lane::BACKWARD || right_lane.direction ==
Lane::UNDIRECTED)
        {
          edge_write_list_.emplace_back(
              EdgeWriteType(lane.full_id.road_id,
lane.full_id.lane_section_id,lane.full_id.lane_id, 1,
                            right_lane.full_id.road_id,
right_lane.full_id.lane_section_id,right_lane.full_id.lane_id, 1,     1));
        }
      }
    }
  }
  std::sort(edge_write_list_.begin(), edge_write_list_.end());
  FILE *fpt = fopen(edges_path.c_str(), "w");

  for (const auto &edge_write : edge_write_list_) {
    fprintf(fpt, "%s %d %d %d %s %d %d %d %d\n",
            edge_write.coming_road_id.c_str(),
            edge_write.coming_laneSection_id,
            edge_write.coming_lane_id,
            edge_write.coming_direction,
            edge_write.going_road_id.c_str(),
            edge_write.going_laneSection_id,
            edge_write.going_lane_id,
            edge_write.going_direction,
            edge_write.type);
  }
  fclose(fpt);
}

void FileWriter::WriteLanesInfo(const FullMap::MapManager &map_manager, const
std::string &lanes_info_path)
{
  lane_info_write_list_.clear();
  for (auto &lane:map_manager.lane_list_.lanes)
  {
    if (lane.type != Lane::DRIVING) continue;
    lane_info_write_list_.emplace_back(LaneInfoWriteType(
        lane.full_id.road_id, lane.full_id.lane_section_id,
        lane.full_id.lane_id, lane.rough_range_normal.min_x,
        lane.rough_range_normal.min_y, lane.rough_range_normal.max_x,
        lane.rough_range_normal.max_y));
  }

  std::sort(lane_info_write_list_.begin(), lane_info_write_list_.end());

  FILE *fpt = fopen(lanes_info_path.c_str(), "w");
  for (const auto &lane_info_write : lane_info_write_list_)
  {
    fprintf(fpt, "%s %d %d %lf %lf %lf %lf\n", lane_info_write.road_id.c_str(),
            lane_info_write.laneSection_id, lane_info_write.lane_id,
            lane_info_write.min_x, lane_info_write.min_y, lane_info_write.max_x,
            lane_info_write.max_y);
  }
  fclose(fpt);
}
*/

/* //tmk_tmp
void FileWriter::WriteRoadside(const FullMap::MapManager &map_manager, const std::string &roadside)
{
  static std::vector<GeometryPolygon> polygons;
  RoadsideWriter *roadside_writer = &RoadsideWriter::GetInstance();
  roadside_writer->Set(map_manager);
  roadside_writer->Get(&polygons);
  FileOperator::Mkdir(roadside);
  FileOperator::DeleleFileBySuffix(roadside, "pcd");
  std::cout << "polygons.size()=" << polygons.size() << std::endl;
  for (int i = 0; i < polygons.size(); i++) {
    std::string filename = roadside + "/" + std::to_string(i) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
  
    // 创建点云  并设置适当的参数（width height is_dense）
    cloud.width    = polygons[i].vertexs.size();
    cloud.height   = 1;
    cloud.is_dense = false;  //不是稠密型的
    cloud.points.resize (cloud.width * cloud.height);  //点云总数大小
    //用随机数的值填充PointCloud点云对象
    for (int j = 0; j < polygons[i].vertexs.size(); j++) {
      cloud.points[j].x = polygons[i].vertexs[j](0);
      cloud.points[j].y = polygons[i].vertexs[j](1);
      cloud.points[j].z = 0;
    }
    //把PointCloud对象数据存储在 test_pcd.pcd文件中
    pcl::io::savePCDFileASCII(filename, cloud);
  
  }
}
*/