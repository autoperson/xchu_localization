// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
#include "union_roadside_builder.h"
#include "src/param_manager/param_manager.h"
#include <fstream>
#include <opendrive_common.h>
#include <chrono>
#include "debug_publisher/debug_publisher.h"
namespace opendrive_generator {
namespace roadside {
/*void OutputLane(const std::string &file_path, const FullMap::MapManager &map_manager,
                const std::string &road_id, const int &laneSection_id,
                const int &lane_id) {
  std::ofstream fout(file_path.c_str());
  for (auto &lane:map_manager.lane_list_.lanes)
  {
    if ( (lane.full_id.road_id!=road_id && road_id!="-1") || (lane.full_id.lane_section_id!=laneSection_id&& laneSection_id!=-1) || (lane.full_id.lane_id!=lane_id)&& (lane_id!=-1))
      continue;
    fout << "------------------------------------------------------------------"
            "-------------"
         << std::endl;
    fout << "full_id:" << lane.full_id.road_id << " "
         << lane.full_id.lane_section_id << " " << lane.full_id.lane_id
         << std::endl;
    fout << "left_line: \n";
    for (auto &node_id : lane.left_line_ids) {
      auto node = map_manager.node_list_.nodes[node_id];
      fout << node_id << " " << node.pose(0) << " " << node.pose(1)
           << std::endl;
    }
    fout << "right_line: \n";
    for (auto &node_id : lane.right_line_ids) {
      auto node = map_manager.node_list_.nodes[node_id];
      fout << node_id << " " << node.pose(0) << " " << node.pose(1)
           << std::endl;
    }
  }
  fout.close();
}
void OutputRoadIDAndPolygonID(const std::string &file_path,const FullMap::MapManager &map_manager)
{
  std::ofstream fout(file_path.c_str());
  fout << "ok" << std::endl;
  
  for (int i = 0; i < map_manager.road_list_.roads.size(); i++) {
    fout << "road_id : " << map_manager.road_list_.roads[i].id
              << std::endl;
    fout << "polyon_id : " << i << std::endl;
    fout << std::endl;
  }
  fout.close();
}
void VistualizePolygon(const std::string &topic_name,const UnionRoadside::Polygon &polygon,const std::vector<UnionRoadside::Node> &node_list)
{
  GeometryPolygon poly;
  
  for (auto &id:polygon.node_id)
  {
    poly.vertexs.emplace_back(node_list[id].pose);
  }
  DebugPublisher *debug_publisher = &DebugPublisher::GetInstance();
  debug_publisher->PublishPolygon(topic_name, poly);
}
void VistualizeAllPolygonLivaPoint(const std::string &topic_name,const std::vector<UnionRoadside::Polygon> &polygons,const std::vector<UnionRoadside::Node> &node_list)
{
  std::vector<GeometryPoint> points;
  for (auto &polygon:polygons)
  {
    for (auto &id:polygon.node_id)
    {
      if (node_list[id].have_del == true) continue;
      points.emplace_back(node_list[id].pose);
    }
  }
  DebugPublisher *debug_publisher = &DebugPublisher::GetInstance();
  debug_publisher->PublishPoints(topic_name, points);
}
void VistualizeAllPolygonEdge(const std::string &topic_name,const std::vector<UnionRoadside::Polygon> &polygons,const std::vector<UnionRoadside::Node> &node_list)
{
  std::vector<GeometryPoint> points;
  for (auto &polygon:polygons)
  {
    int j = polygon.node_id.back();
    for (auto &i : polygon.node_id)
    {
      points.emplace_back(node_list[j].pose);
      points.emplace_back(node_list[i].pose);
      j=i;
      
    }
  }
  DebugPublisher *debug_publisher = &DebugPublisher::GetInstance();
  debug_publisher->PublishEdges(topic_name, points);
}
*/
void VistualizeAllPolygonLivaEdge(const std::string &topic_name,const std::vector<union_roadside::Polygon> &polygons,const std::vector<union_roadside::Node> &node_list)
{
  std::vector<opendrive_common::GeometryPoint> points;
  for (int i = 0; i < polygons.size();i++) {
    
    auto &polygon = polygons[i];
    for (auto &id:polygon.node_id)
    {
      if (node_list[id].have_del == true) continue;
      for (auto &edge:node_list[id].edges)
      {
        //if (edge.belong_polygon_id!=i)
        {
          points.emplace_back(node_list[id].pose);
          points.emplace_back(node_list[edge.to_node_id].pose);
        }
      }
    }
  }
  DebugPublisher *debug_publisher = &DebugPublisher::GetInstance();
  debug_publisher->PublishEdges(topic_name, points);
}
void UnionRoadsideBuilder::Build(const hdmap::EntireMap &entire_map, std::vector<opendrive_common::GeometryPolygon> *output_polygons)
{
  auto begin = std::chrono::high_resolution_clock::now();
  output_polygons->clear();
  //OutputLane(base_debug_path+"601.txt",map_manager,"6",0,1);
  //OutputLane(base_debug_path+"602.txt",map_manager,"6",0,2);
  //OutputLane(base_debug_path+"6xx.txt",map_manager,"6",-1,-1);
  BuildOriginPolygon(entire_map, &polygons_, &node_list_);
 
  // if (debug_mode)
  //VistualizeAllPolygonLivaEdge("/vistualize/debug/edges",polygons_,node_list_);
  // if (debug_mode)
  //  OutputRoadIDAndPolygonID(base_debug_path +
  //  "/road_id_and_road_list_id.txt", map_manager);
  BuildPolygonRoughInfo(node_list_,&polygons_);
  BuildPolygonSortEdge(node_list_, &polygons_);
  //if (debug_mode)
  //  VistualizeAllPolygonEdge("vistualize/debug/all_polygon_edge", polygons_,
  //                           node_list_);
  //
  //if (debug_mode) OutputDebugInfo(base_debug_path+"before_first_delete.txt",polygons_,node_list_);
  DeleteOverlapOriginalNode( polygons_, &node_list_);
  //if (debug_mode)VistualizeAllPolygonLivaEdge("/vistualize/debug/edges",polygons_,node_list_);
  //if (debug_mode) OutputDebugInfo(base_debug_path+"before_insert.txt",polygons_,node_list_);
  InsertIntersectPoint(&polygons_,&node_list_);
  //if (debug_mode) OutputDebugInfo(base_debug_path+"/before_delete.txt",polygons_,node_list_);
  
  DeleteOverlapNode( polygons_, &node_list_);
  DeleteOverlapIntersectionNode( polygons_, &node_list_);
  
  DeleteOverlapEdge(polygons_,&node_list_);

  //if (debug_mode) OutputDebugInfo(base_debug_path+"after_delete.txt",polygons_,node_list_);
  //if (debug_mode)VistualizeAllPolygonLivaPoint("/vistualize/debug/all_polygons",polygons_,node_list_);
  
  BuildOutputPolygon(polygons_,&node_list_,output_polygons);
  
  PolygonPointReduction(output_polygons);
  //std::cout << output_polygons->size() << std::endl;
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double,std::ratio<1,1>> duration_s(end-begin);//end-begin得到一个duration类型
  std::cout<<"[UnionRoadsideBuilder::Build]: Cost " <<duration_s.count()<< " seconds"<<std::endl;
}


/*
void UnionRoadsideBuilder::OutputDebugInfo(
    const std::string &file_path,
    const std::vector<UnionRoadside::Polygon> polygons,
    const std::vector<UnionRoadside::Node> &node_list) {
  std::ofstream fout(file_path.c_str());
  fout << "polygon:\n";

  for (int i=0;i<polygons.size();i++)
  {
    fout << "\n\n";
    fout << "p_id " << i << std::endl;
    fout << "corner node id:" << std::endl;
    fout << polygons[i].left_front_id << " " << polygons[i].right_front_id
         << std::endl;
    fout << polygons[i].left_back_id << " " << polygons[i].right_back_id << std::endl;
    fout << polygons[i].rough_range_normal.min_x << " "
         << polygons[i].rough_range_normal.max_x << " "
         << polygons[i].rough_range_normal.min_y << " "
         << polygons[i].rough_range_normal.max_y << std::endl;
    fout << "outline_lane_id:" << std::endl;
    for (auto lane_pair:polygons[i].outline_lane_id)
    {
      fout << lane_pair.first << " " << lane_pair.second << std::endl;
    }
    fout << "all node_id:" << std::endl;
    for (auto &node_id : polygons[i].node_id) {
      fout << node_id << std::endl;
    }

    fout << "sort_edge:" << std::endl;
    for (auto &edge : polygons[i].sort_edge) {
      fout << edge.id[0] << " " <<  edge.id[1] << " " << edge.min_y0 << std::endl;
    }
    fout << "\n\n";
    
  }
  fout <<"\n\n";
  fout << "node_id:\n";
  for (int i=0;i<node_list.size();i++)
  {
    fout << i << " " << "[" << node_list[i].pose(0) << " "<<node_list[i].pose(1)  << "] ";
    for (auto edge:node_list[i].edges)
    {
      fout <<"("<<edge.to_node_id << ", " << edge.belong_polygon_id <<") ";
    }
    fout << "\n";
  }
  fout.close();
}
*/

void UnionRoadsideBuilder::BuildOriginPolygon(const hdmap::EntireMap &entire_map, std::vector<union_roadside::Polygon>  *polygons,std::vector<union_roadside::Node> *node_list)
{
  std::cout << "[1/7][BuildOriginPolygon]: start" << std::endl;
  polygons->clear();
  node_list->clear();
  auto &road_list = entire_map.road_list();
  polygons->resize(road_list.size());
  std::cout << road_list.size() << std::endl;
  for (int i = 0; i < road_list.size(); i++) {
    BuildOutlineLaneID( road_list[i],
                       &((*polygons)[i].outline_lanes));
    /*std::cout << "outline_lane_id polygon_id:" << i << std::endl;
    for (auto lane_id_pair : (*polygons)[i].outline_lane_id)
    {
      std::cout << "\n";
      std::cout << map_manager.lane_list_.lanes.size() << "\n";
      std::cout
          << "left=" << lane_id_pair.first << " ("
          << map_manager.lane_list_.lanes[lane_id_pair.first].full_id.road_id
          << " "
          << map_manager.lane_list_.lanes[lane_id_pair.first]
                 .full_id.lane_section_id
          << " "
          << map_manager.lane_list_.lanes[lane_id_pair.first].full_id.lane_id
          << "\n";
      std::cout
          << "right=" << lane_id_pair.second << " ("
          << map_manager.lane_list_.lanes[lane_id_pair.second].full_id.road_id
          << " "
          << map_manager.lane_list_.lanes[lane_id_pair.second]
                 .full_id.lane_section_id
          << " "
          << map_manager.lane_list_.lanes[lane_id_pair.second].full_id.lane_id
          << "\n";
    }
    */
    BuildSingleRoad(road_list[i],
                    (*polygons)[i].outline_lanes, i, &((*polygons)[i]), node_list);
  }
  BuildConnection(entire_map,*polygons,node_list);
  
}
void UnionRoadsideBuilder::BuildOutlineLaneID(const hdmap::RoadPtr &road,std::vector<std::pair<std::weak_ptr<hdmap::Lane>, std::weak_ptr<hdmap::Lane>> > *outline_lanes)
{
  outline_lanes->clear();
  for (auto &laneSection:road->laneSections)
  {
    int left = 0;
    int right = laneSection->lanes.size()-1;
    while (left<laneSection->lanes.size())
    {
      auto &lane = laneSection->lanes[left];
      if (lane->type == hdmap::Lane::Type::kDriving) break;
      left++;
    }
    while (right>=0)
    {
      auto &lane = laneSection->lanes[right];
      if (lane->type == hdmap::Lane::Type::kDriving) break;
      right--;
    }
    if (left>=laneSection->lanes.size()) continue;
    outline_lanes->emplace_back(std::make_pair(laneSection->lanes[left],laneSection->lanes[right]));
  }
}
void UnionRoadsideBuilder::BuildSingleRoad(const hdmap::RoadPtr &road, const std::vector< std::pair<std::weak_ptr<hdmap::Lane>, std::weak_ptr<hdmap::Lane> > >  &outline_lanes, const int &polygon_id,union_roadside::Polygon *polygon,std::vector<union_roadside::Node>  *node_list)
{
  polygon->belong_junction = road->belong_junction;
  polygon->node_id.clear();
  int polygon_point_num=0;
  
  for (int i=0;i<outline_lanes.size();i++)
  {
    polygon_point_num+=outline_lanes[i].first.lock()->lane_nodes.size();
    polygon_point_num+=outline_lanes[i].second.lock()->lane_nodes.size();
  }
  int base_node_num=node_list->size();
  node_list->resize(base_node_num+polygon_point_num);
  polygon->node_id.clear();
  int now_num=0;

  int left_front_id=base_node_num;
  int last_node_id = -1;
  for (int i=outline_lanes.size()-1;i>=0;i--)
  {
    auto left_lane = outline_lanes[i].first.lock();
    for (int node_id_in_left_line =
             left_lane->lane_nodes.size() - 1;
         node_id_in_left_line >= 0; node_id_in_left_line--) {
      auto &lane_node = left_lane->lane_nodes[node_id_in_left_line];
      auto &line_node = lane_node->left_line_node;
      int now_node_id = base_node_num + now_num;
      (*node_list)[now_node_id].Init(line_node->pose,polygon_id);
      polygon->node_id.emplace_back(now_node_id);
      if (last_node_id!=-1)
      {
        AddUndirectedEdge(last_node_id,now_node_id,polygon_id,node_list);
      }
      last_node_id = now_node_id;
      now_num++;
    }
  }
  int left_back_id=base_node_num+now_num-1;
  int right_back_id=base_node_num+now_num;

  last_node_id = -1;
  for (int i=0;i<outline_lanes.size();i++)
  {
    auto right_lane = outline_lanes[i].second.lock();
    for (int node_id_in_right_line = 0;
         node_id_in_right_line <right_lane->lane_nodes.size();node_id_in_right_line++) {
      auto &lane_node = right_lane->lane_nodes[node_id_in_right_line];
      auto &line_node = lane_node->right_line_node;
      int now_node_id = base_node_num+now_num;
      (*node_list)[now_node_id].Init(line_node->pose,polygon_id);
      polygon->node_id.emplace_back(now_node_id);

      if (last_node_id!=-1)
      {
        AddUndirectedEdge(last_node_id,now_node_id,polygon_id,node_list);
      }
      last_node_id = now_node_id;
      now_num++;  
    }
  }
  int right_front_id=base_node_num+now_num-1;  
  polygon->left_front_id=left_front_id;
  polygon->left_back_id=left_back_id;
  polygon->right_back_id=right_back_id;
  polygon->right_front_id=right_front_id;

  if (road->predecessor_road_edges.empty())
  {
    puts("predecessor");
    AddUndirectedEdge(polygon->left_back_id,polygon->right_back_id,polygon_id,node_list);
  }


  if (road->successor_road_edges.empty())
  {
    puts("successor");
    AddUndirectedEdge(polygon->left_front_id,polygon->right_front_id,polygon_id,node_list);
  }

}


void UnionRoadsideBuilder::BuildConnection(const hdmap::EntireMap &entire_map,const std::vector<union_roadside::Polygon>  &polygon,std::vector<union_roadside::Node>  *node_list)
{
  auto &road_list = entire_map.road_list();
  for (int from = 0; from < road_list.size(); from++) {
    auto &from_road=road_list[from];
    int from_node_id_1, from_node_id_2;
    from_node_id_1 = polygon[from].left_back_id;
    from_node_id_2 = polygon[from].right_back_id;
    for (auto &road_edge:from_road->predecessor_road_edges)
    {
      int go=road_edge.connecting_road.lock()->list_id;
      int go_node_id_1,go_node_id_2;
      bool if_add_1=false, if_add_2 = false;
      if (road_edge.contact_point==hdmap::RoadEdge::ContactPoint::kStart)
      {
        //if (entire_map.CheckIfPredecessor(polygon[from].outline_lane_id.front().first,polygon[go].outline_lane_id.front().second))
        if (polygon[from].outline_lanes.front().first.lock()->CheckIfPredecessor(polygon[go].outline_lanes.front().second.lock()))
        {
          if_add_1=true;
          go_node_id_1 = polygon[go].right_back_id;
        }
        //if (.lock()->CheckIfPredecessor(.lock()))

        //if (entire_map.CheckIfPredecessor(polygon[from].outline_lane_id.front().second,polygon[go].outline_lane_id.front().first))
        if (polygon[from].outline_lanes.front().second.lock()->CheckIfPredecessor(polygon[go].outline_lanes.front().first.lock()))
        {
          if_add_2=true;
          go_node_id_2 = polygon[go].left_back_id;
        }
      }
      else
      if (road_edge.contact_point==hdmap::RoadEdge::ContactPoint::kEnd)
      {
        
        //if (entire_map.CheckIfPredecessor(polygon[from].outline_lane_id.front().first,polygon[go].outline_lane_id.back().first))
        if (polygon[from].outline_lanes.front().first.lock()->CheckIfPredecessor(polygon[go].outline_lanes.back().first.lock()))
        {
          if_add_1=true;
          go_node_id_1 = polygon[go].left_front_id;
        }
        //if (entire_map.CheckIfPredecessor(polygon[from].outline_lane_id.front().second,polygon[go].outline_lane_id.back().second))
        if (polygon[from].outline_lanes.front().second.lock()->CheckIfPredecessor(polygon[go].outline_lanes.back().second.lock()))
        {
          if_add_2=true;
          go_node_id_2 = polygon[go].right_front_id;
        }
      }
      else
      {
        opendrive_common::Output::Error("[UnionRoadsideBuilder::BuildConnection]: edge contact point error.");
      }
      if (if_add_1)
      {
        (*node_list)[from_node_id_1].AddEdge(go_node_id_1,go);
        (*node_list)[go_node_id_1].AddEdge(from_node_id_1,from);
      }
      if (if_add_2)
      {
        (*node_list)[from_node_id_2].AddEdge(go_node_id_2,go);
        (*node_list)[go_node_id_2].AddEdge(from_node_id_2,from);
      }
    }

    from_node_id_1 = polygon[from].left_front_id;
    from_node_id_2 = polygon[from].right_front_id;
    for (auto &road_edge:from_road->successor_road_edges)
    {
      int go=road_edge.connecting_road.lock()->list_id;
      int go_node_id_1,go_node_id_2;
      bool if_add_1=false, if_add_2=false;
      if (road_edge.contact_point==hdmap::RoadEdge::ContactPoint::kStart)
      {
        //if (entire_map.CheckIfSuccessor(polygon[from].outline_lane_id.back().first,polygon[go].outline_lane_id.front().first))
        if (polygon[from].outline_lanes.back().first.lock()->CheckIfSuccessor(polygon[go].outline_lanes.front().first.lock()))
        {
          if_add_1=true;
          go_node_id_1 = polygon[go].left_back_id;
        }
        //if (entire_map.CheckIfSuccessor(polygon[from].outline_lane_id.back().second,polygon[go].outline_lane_id.front().second))
        if (polygon[from].outline_lanes.back().second.lock()->CheckIfSuccessor(polygon[go].outline_lanes.front().second.lock()))
        {
          if_add_2 = true;
          go_node_id_2 = polygon[go].right_back_id;
        }
      }
      else
      if (road_edge.contact_point==hdmap::RoadEdge::ContactPoint::kEnd)
      {
        //if (entire_map.CheckIfSuccessor(polygon[from].outline_lane_id.back().first,polygon[go].outline_lane_id.back().second))
        if (polygon[from].outline_lanes.back().first.lock()->CheckIfSuccessor(polygon[go].outline_lanes.back().second.lock()))
        {
          if_add_1=true;
          go_node_id_1 = polygon[go].right_front_id;
        }
        //if (entire_map.CheckIfSuccessor(polygon[from].outline_lane_id.back().second,polygon[go].outline_lane_id.back().first))
        if (polygon[from].outline_lanes.back().second.lock()->CheckIfSuccessor(polygon[go].outline_lanes.back().first.lock()))
        {
          if_add_2=true;
          go_node_id_2 = polygon[go].left_front_id;
        }
      }
      else
      {
        opendrive_common::Output::Error("[UnionRoadsideBuilder::BuildConnection]: edge contact point error.");
      }
      if (if_add_1)
      {
        (*node_list)[from_node_id_1].AddEdge(go_node_id_1,go);
        (*node_list)[go_node_id_1].AddEdge(from_node_id_1,from);
      }
      if (if_add_2)
      {
        (*node_list)[from_node_id_2].AddEdge(go_node_id_2,go);
        (*node_list)[go_node_id_2].AddEdge(from_node_id_2,from);
      }
    }
  }
}


/*
void UnionRoadsideBuilder::AddEdgeDifferentRoad(const std::vector<UnionRoadside::Polygon>  &polygon,const int &from,const int &go,const Road::BuildInfo::Link::ContactPoint &from_contact_point,const Road::BuildInfo::Link::ContactPoint &go_contact_point,const ConnectionInfo &conection_info,std::vector<UnionRoadside::Node>  *node_list)
{
  int from_id_1,from_id_2;
  int go_id_1,go_id_2;
  bool if_connect_1 = false;
  bool if_connect_2 = false;
  if (polygon[from].belong_junction=="-1" && polygon[go].belong_junction=="-1")
  {
    if_connect_1 = if_connect_2 = true;
  }
  if (from_contact_point == FullMap::Road::BuildInfo::Link::kStart) {
    from_id_1 = polygon[from].left_back_id;
    from_id_2 = polygon[from].right_back_id;
    if (go_contact_point==FullMap::Road::BuildInfo::Link::kStart)
    {
      go_id_1 = polygon[go].right_back_id;
      go_id_2 = polygon[go].left_back_id;
      if (!if_connect_1)
      {
        for (auto &lane_id_pair:conection_info.lane_id_pair)
        {
          if (lane_id_pair.first==polygon[from].outline_lane_id.front().first && lane_id_pair.second==polygon[go].outline_lane_id.front().second)
          {
            if_connect_1 = true;
          }
          if (lane_id_pair.first==polygon[from].outline_lane_id.front().second && lane_id_pair.second==polygon[go].outline_lane_id.front().first)
          {
            if_connect_2 = true;
          }
        }
      }
    }
    else
    if (go_contact_point==FullMap::Road::BuildInfo::Link::kEnd)
    {
      go_id_1 = polygon[go].left_front_id;
      go_id_2 = polygon[go].right_front_id;
      if (!if_connect_1)
      {
        for (auto &lane_id_pair:conection_info.lane_id_pair)
        {
          if (lane_id_pair.first==polygon[from].outline_lane_id.front().first && lane_id_pair.second==polygon[go].outline_lane_id.back().first)
          {
            if_connect_1 = true;
          }
          if (lane_id_pair.first==polygon[from].outline_lane_id.front().second && lane_id_pair.second==polygon[go].outline_lane_id.back().second)
          {
            if_connect_2 = true;
          }
        }
      }
    }
    else
    {
      ROS_ERROR("[UnionRoadsideBuilder::AddEdgeDifferentRoad]: go_contact_point error1.");
      return ;
    }
    
  }
  else
  if (from_contact_point==FullMap::Road::BuildInfo::Link::kEnd)
  {
    from_id_1 = polygon[from].left_front_id;
    from_id_2 = polygon[from].right_front_id;
    if (go_contact_point==FullMap::Road::BuildInfo::Link::kStart)
    {
      go_id_1 = polygon[go].left_back_id;
      go_id_2 = polygon[go].right_back_id;
      if (!if_connect_1)
      {
        for (auto &lane_id_pair:conection_info.lane_id_pair)
        {
          if (lane_id_pair.first==polygon[from].outline_lane_id.back().first && lane_id_pair.second==polygon[go].outline_lane_id.front().first)
          {
            if_connect_1 = true;
          }
          if (lane_id_pair.first==polygon[from].outline_lane_id.back().second && lane_id_pair.second==polygon[go].outline_lane_id.front().second)
          {
            if_connect_2 = true;
          }
        }
      }
    }
    else
    if (go_contact_point==FullMap::Road::BuildInfo::Link::kEnd)
    {
      go_id_1 = polygon[go].right_front_id;
      go_id_2 = polygon[go].left_front_id;
       if (!if_connect_1)
      {
        for (auto &lane_id_pair:conection_info.lane_id_pair)
        {
          if (lane_id_pair.first==polygon[from].outline_lane_id.back().first && lane_id_pair.second==polygon[go].outline_lane_id.back().second)
          {
            if_connect_1 = true;
          }
          if (lane_id_pair.first==polygon[from].outline_lane_id.back().second && lane_id_pair.second==polygon[go].outline_lane_id.back().first)
          {
            if_connect_2 = true;
          }
        }
      }
    }
    else
    {
      ROS_ERROR("[UnionRoadsideBuilder::AddEdgeDifferentRoad]: go_contact_point error1.");
      return ;
    }
  }
  else
  {
    ROS_ERROR("[UnionRoadsideBuilder::AddEdgeDifferentRoad]: from_contact_point error.");
    return ;
  }
  // std::cout << "from:" << from << std::endl;
  // std::cout << "go:" << go << std::endl;
  // std::cout << "from_contact_point:" << from_contact_point << std::endl;
  // std::cout << "go_contact_point:" << go_contact_point << std::endl;
  // std::cout << "from_id_1:" << from_id_1 << std::endl;
  // std::cout << "go_id_1:" << go_id_1 << std::endl;
  // std::cout << "from_id_2:" << from_id_2 << std::endl;
  // std::cout << "go_id_2:" << go_id_2 << std::endl;
  // std::cout << "if_connect_1:" <<  if_connect_1 << std::endl;
  // std::cout << "if_connect_2:" <<  if_connect_2 << std::endl;
  if (if_connect_1)
  {
    (*node_list)[from_id_1].AddEdge(go_id_1,go);
    (*node_list)[go_id_1].AddEdge(from_id_1,from);
  }
  if (if_connect_2)
  {
    (*node_list)[from_id_2].AddEdge(go_id_2,go);
    (*node_list)[go_id_2].AddEdge(from_id_2,from);
  }


}
*/
void UnionRoadsideBuilder::BuildPolygonRoughInfo(const std::vector<union_roadside::Node>  &node_list,std::vector<union_roadside::Polygon>  *polygons)
{
  std::cout << "[2/7][BuildPolygonRoughInfo]: start" << std::endl;
  for (auto &polygon:*polygons)
  {
    double max_x=-1e+100;
    double min_x=1e+100;
    double max_y=-1e+100;
    double min_y=1e+100;
    for (auto &node_id:polygon.node_id)
    {
      max_x=std::max(max_x,node_list[node_id].pose(0));
      min_x=std::min(min_x,node_list[node_id].pose(0));
      max_y=std::max(max_y,node_list[node_id].pose(1));
      min_y=std::min(min_y,node_list[node_id].pose(1));
    }
    max_x += 1e-6;
    min_x -= 1e-6;
    max_y += 1e-6;
    min_y -= 1e-6;
    polygon.rough_range_normal.SetData(max_x, min_x, max_y, min_y);
  }
}

void UnionRoadsideBuilder::InsertIntersectPoint(std::vector<union_roadside::Polygon>  *polygons,std::vector<union_roadside::Node>  *node_list)
{
  std::cout << "[3/7][InsertIntersectPoint]: start" << std::endl;
  auto &param = ParamManager::GetInstance().GetParam();
  double min_segment_length=param.union_roadside_builder().min_segment_length();
  int insert_num = 0;
  int check_num = 0;
  for (int i = 0; i < polygons->size(); i++) {  
    std::cout << "[3/7][InsertIntersectPoint]: Dealing " << i <<"/ " << polygons->size()<< std::endl;
    auto &main_polygon = (*polygons)[i];
    if (main_polygon.belong_junction == "-1") continue;
    for (int j = i + 1; j < polygons->size(); j++) {
      std::cout<<"Checking polygon_" << i << " which size is " << (*polygons)[i].node_id.size() << " and polygon_" << j << " which size is " << (*polygons)[j].node_id.size() << std::endl;
      auto &check_polygon = (*polygons)[j];
      if (!main_polygon.rough_range_normal.IfIntersect(check_polygon.rough_range_normal))  continue;
      if (main_polygon.belong_junction != check_polygon.belong_junction) continue;
      int last_main_node_id = *(main_polygon.node_id.rbegin());
      for (auto main_node_id_iter=main_polygon.node_id.begin();main_node_id_iter!=main_polygon.node_id.end();main_node_id_iter++)
      {
        int now_main_node_id = *main_node_id_iter;
        int main_edge_id;
        if (!(*node_list)[last_main_node_id].FindEdge(now_main_node_id, i,&main_edge_id)) {
          last_main_node_id = now_main_node_id;
          continue;
        }
        auto &main_edge =(*node_list)[last_main_node_id].edges[main_edge_id];

        if (!(main_edge.is_intersect)) 
        {
          last_main_node_id = now_main_node_id;
          continue;
        }

        opendrive_common::GeometryLineSegment main_segment((*node_list)[last_main_node_id].pose,
                                         (*node_list)[now_main_node_id].pose);
        if (main_segment.GetLength()<min_segment_length) 
        {
          last_main_node_id = now_main_node_id;
          continue;
        }

        int last_check_node_id = *(check_polygon.node_id.rbegin());
        std::vector<int> insert_node_ids;
        insert_node_ids.clear();
        for (auto check_node_id_iter = check_polygon.node_id.begin();
             check_node_id_iter != check_polygon.node_id.end();
             check_node_id_iter++) {
          int now_check_node_id = *(check_node_id_iter);
          int check_edge_id;
          if (!(*node_list)[last_check_node_id].FindEdge(now_check_node_id,j,&check_edge_id))
          {
            last_check_node_id = now_check_node_id;
            continue;
          }
          auto &check_edge=(*node_list)[last_check_node_id].edges[check_edge_id];
          if (!check_edge.is_intersect) 
          {
            last_check_node_id = now_check_node_id;
            continue;
          } 
          opendrive_common::GeometryLineSegment check_segment( (*node_list)[last_check_node_id].pose,(*node_list)[now_check_node_id].pose);
          if (check_segment.GetLength()<min_segment_length) 
          {
            last_check_node_id = now_check_node_id;
            continue;
          }
          std::vector<opendrive_common::GeometryPoint> intersections;
          int res=opendrive_common::GetLineSegemnt2LineSegmentIntersection(main_segment,check_segment,&intersections);
          check_num++;
          if (res == 0 || res == 2) {
            last_check_node_id = now_check_node_id;
            continue;
          }

          if (intersections[0](0)==(*node_list)[last_main_node_id].pose(0) && intersections[0](1)==(*node_list)[last_main_node_id].pose(1)){
            last_check_node_id = now_check_node_id;
            continue;
          }
          if (intersections[0](0) == (*node_list)[last_check_node_id].pose(0) &&
              intersections[0](1) == (*node_list)[last_check_node_id].pose(1)) {
            last_check_node_id = now_check_node_id;
            continue;
          }
          union_roadside::Node node;
          node.Init(intersections[0],i,j);
          node.is_intersection_point = true;
          node_list->emplace_back(node);
          insert_num++;
          int insert_node_id = node_list->size() - 1;
          check_polygon.node_id.insert(check_node_id_iter, insert_node_id);
          insert_node_ids.emplace_back(insert_node_id);
          check_node_id_iter++;

          RemoveUndirectedEdge(now_check_node_id, last_check_node_id,
                               node_list);
          // std::cout << " Add edge in j " << j  << std::endl;
          // std::cout << "insert " << last_check_node_id << " to "
          //           << insert_node_id << std::endl;
          AddUndirectedEdge(last_check_node_id, insert_node_id, j, node_list);
          // std::cout << "insert " << insert_node_id << " to "
          //           << now_check_node_id << std::endl;
          AddUndirectedEdge(insert_node_id,now_check_node_id,j,node_list);

          last_check_node_id = now_check_node_id;
        }
        if (insert_node_ids.size()!=0)
        {
          std::sort(insert_node_ids.begin(), insert_node_ids.end(),
              [&](const int &x, const int &y) {
                double len_0= opendrive_common::GeometryLineSegment( (*node_list)[last_check_node_id].pose,(*node_list)[x].pose).GetLength();
                double len_1= opendrive_common::GeometryLineSegment( (*node_list)[last_check_node_id].pose,(*node_list)[y].pose).GetLength();
                return len_0 < len_1;
              });
          RemoveUndirectedEdge(last_main_node_id,now_main_node_id,node_list);
          int last_insert_node_id=last_main_node_id;
          
          for (auto now_insert_node_id : insert_node_ids) {
            // std::cout << " Add edge in i " << i << std::endl;
            main_polygon.node_id.insert(main_node_id_iter, now_insert_node_id);  
            // std::cout << "insert " << last_insert_node_id << " to "
            //         << now_insert_node_id << std::endl;
            AddUndirectedEdge(last_insert_node_id,now_insert_node_id,i,node_list);
            last_insert_node_id = now_insert_node_id;
            main_node_id_iter++;
          }

          // std::cout << "insert " << last_insert_node_id << " to "
          //           << now_main_node_id << std::endl;
          AddUndirectedEdge(last_insert_node_id,now_main_node_id,i,node_list);
        }
        
        last_main_node_id = now_main_node_id;

      }

    }
  }
  std::cout << "[3/7][InsertIntersectPoint]: Check "<< check_num <<" times intersection. Insert "<< insert_num << " nodes successful." << std::endl;
}

int calc_num;

void UnionRoadsideBuilder::RemoveUndirectedEdge(const int &x,const int &y,std::vector<union_roadside::Node> *node_list)
{
  (*node_list)[x].RemoveEdge(y);
  (*node_list)[y].RemoveEdge(x);
}

void UnionRoadsideBuilder::AddUndirectedEdge(const int &x,const int &y,const int &polygon_id,std::vector<union_roadside::Node> *node_list)
{
  (*node_list)[x].AddEdge(y,polygon_id);
  (*node_list)[y].AddEdge(x,polygon_id);
}

void UnionRoadsideBuilder::DeleteOverlapNode(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node>  *node_list)
{
  std::cout << "[4/7][DeleteOverlapNode]: start"<< std::endl;
  calc_num=0;
  int delete_num = 0;
  for (int i = 0; i < polygons.size(); i++) {
    std::cout << "[4/7][DeleteOverlapNode]: " <<i << " " <<"/ " << polygons.size() << std::endl;
    auto &main_polygon = polygons[i];
    if (main_polygon.belong_junction == "-1" ) continue;
    for (int j=0;j<polygons.size();j++)
    {
      if (i==j) continue;
      auto &check_polygon = polygons[j];
      if (!main_polygon.rough_range_normal.IfIntersect(check_polygon.rough_range_normal))  continue;
      if (main_polygon.belong_junction != check_polygon.belong_junction) continue;
      std::cout<<"Checking polygon_" << i << " which size is " << polygons[i].node_id.size() << " and polygon_" << j << " which size is " << polygons[j].node_id.size() << std::endl;
      for (auto &main_node_id:main_polygon.node_id)
      {
        if ((*node_list)[main_node_id].have_del) continue;
        if (!check_polygon.rough_range_normal.IfInRoughRange((*node_list)[main_node_id].pose)) continue;
        if (FastPointIsInPolygon((*node_list)[main_node_id].pose,polygons[j],*node_list))
        {
          (*node_list)[main_node_id].have_del=true;
          delete_num++;
        }
      }
    }
    
  }
  std::cout << "[4/7][InsertIntersectPoint]: Total calculate " <<calc_num<<" times for intersections. Delete "<< delete_num << " nodes successful." << std::endl; 
}



void UnionRoadsideBuilder::DeleteOverlapEdge(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node>  *node_list)
{
  std::cout << "[5/7][DeleteOverlapEdge]: start"<< std::endl;
  for (auto &node:*node_list)
  {
    if (node.have_del) node.edges.clear();
    else
    for (auto edge_iter=node.edges.begin();edge_iter!=node.edges.end();)
    {
      if ((*node_list)[edge_iter->to_node_id].have_del)
      {
        edge_iter=node.edges.erase(edge_iter);
      }
      else
      {
        edge_iter++;
      }      
    }
  }
}

void UnionRoadsideBuilder::BuildOutputPolygon(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node> *node_list,std::vector<opendrive_common::GeometryPolygon> *output_polygons)
{
  std::cout << "[6/7][BuildOutputPolygon]: start"<< std::endl;
  // std::ofstream fout(base_debug_path+"roadside_polygon.txt");
  std::vector<opendrive_common::GeometryPoint> delete_point;
  delete_point.clear();
  for (int i = 0; i < node_list->size(); i++) {
    if ((*node_list)[i].have_del) continue;
    if ((*node_list)[i].edges.size()!=2) continue;
    std::vector<opendrive_common::GeometryPoint> now_delete_point;
    now_delete_point.clear();
    opendrive_common::GeometryPolygon polygon;
    polygon.vertexs.clear();
    std::list<std::pair<int,int > > sta;
    sta.clear();
    sta.push_back(std::make_pair(i, 0));
    node_list_[i].have_in_stack = true;
    now_delete_point.emplace_back(node_list_[i].pose);
    // if (i == 0) fout << "push  " << " [" << node_list_[0].pose(0) << ", " << node_list_[0].pose(1)<< std::endl;
    while (!sta.empty()) {

      auto now = sta.back();
      std::cout << "stack.size()= "<<sta.size()<<", stack top is " << now.first << " " <<  now.second<< std::endl;
      int node_id = now.first;
      int edge_id = now.second;
      int last_node_id = -1;
      auto iter = sta.rbegin();
      iter++;
      if (iter!=sta.rend()) {
        last_node_id = iter->first;
      }
      for (; edge_id < node_list_[node_id].edges.size(); edge_id++) {
        int to_node_id = node_list_[node_id].edges[edge_id].to_node_id;
        if (to_node_id == last_node_id) continue;
        if (node_list_[to_node_id].have_del) continue;
        break;
      }

      if (edge_id >= node_list_[node_id].edges.size())
      {
        node_list_[node_id].have_del = true;
        node_list_[node_id].have_in_stack = false;
        sta.pop_back();
        if (!sta.empty())
        {
          std::cout << "go back to  " << sta.back().first << std::endl;
          // if (i == 0) fout << "go back to  " << sta.back().first << std::endl;
          sta.back().second++;
        }
        continue;
      }
      sta.back().second=edge_id;
      int next_node_id = node_list_[node_id].edges[edge_id].to_node_id;
      if (node_list_[next_node_id].have_in_stack == false) {
        std::cout << "push  " << next_node_id << std::endl;
        // if (i == 0) fout << "push  " << next_node_id << " [" << node_list_[next_node_id].pose(0) << ", " << node_list_[next_node_id].pose(1)<< std::endl;
        now_delete_point.emplace_back(node_list_[next_node_id].pose);
        sta.push_back(std::make_pair(next_node_id, 0));
        node_list_[next_node_id].have_in_stack = true;
        continue;
      }
      else
      {
        
        // if (i==0)
        // {
        //   fout << "Build polygon" << std::endl;
        //   fout << "sta.size()=" << sta.size() << std::endl;
        // }
        polygon.vertexs.clear();
        bool go_continue = false;
        for (auto iter = sta.rbegin(); iter != sta.rend(); iter++) {
          auto &now_element = *iter;
          if (now_element.first == next_node_id) {
            go_continue = true;
            if (polygon.vertexs.size() > 2) {
              double S = opendrive_common::GetPolygonAbsoluteArea(polygon);
              // if (i == 0) fout << "S=" << S << std::endl;
              std::cout << "S=" << S << std::endl;
              if (S > 0.1) {
                go_continue = false;
              }
            }
            break;
          }
          if (polygon.vertexs.size() == 0 ||
              opendrive_common::GeometryLineSegment(polygon.vertexs.back(),
                                  node_list_[now_element.first].pose)
                      .GetLength() > 1e-6) {
            polygon.vertexs.emplace_back(node_list_[now_element.first].pose);
          }
        }
        std::cout << "go_continue=" << go_continue << std::endl;
        if (go_continue) {
          sta.back().second++;
          polygon.vertexs.clear();
          // if (i==0)
            // fout << "Continue" << std::endl;
          continue;
        }
        // if (i==0)
        //     fout << "have answer" << std::endl;
        for (auto &node_pair:sta)
        {
          (*node_list)[node_pair.first].have_del = true;
          (*node_list)[node_pair.first].have_in_stack = false;
        }
        sta.clear();
      }
    }
    while (polygon.vertexs.size()>1)
    {
      if (opendrive_common::GeometryLineSegment(polygon.vertexs.front(),polygon.vertexs.back()).GetLength()>1e-6)
        break;
      polygon.vertexs.pop_back();
    }
    // fout << "polygon_" << output_polygons->size() << ": S=" << geometry_2d::GetPolygonAbsoluteArea(polygon) << std::endl;
    // for (auto &point : polygon.vertexs) {
    //   fout << point(0) << " "<< point(1) << std::endl;
    // }
    // fout << "\n\n";
    for (auto &point : now_delete_point) delete_point.emplace_back(point);
    if (polygon.vertexs.size() == 0) {
      std::cout << "[6/7][BuildOutputPolygon]: find a empty polygon";
      
    } else {
      std::cout << "[6/7][BuildOutputPolygon]: polygon_" << output_polygons->size() << " have generated "<<polygon.vertexs.size() << "points";
      output_polygons->emplace_back(polygon);
    }
  }
  
  std::cout << "[6/7][BuildOutputPolygon] : Generate " << output_polygons->size()
            << " polygon successful" << std::endl;

  // fout.close();
}

void UnionRoadsideBuilder::AddEdgeJunction(
    const std::vector<union_roadside::Polygon> &polygons, const int &from,
    const std::list<ConnectionInfo> &list_connecting_info,
    const hdmap::Road::BuildInfo::Link::ContactPoint &from_contact_point,
    std::vector<union_roadside::Node> *node_list)
{
  for (auto &connecting_info:list_connecting_info)
  {
    int go = connecting_info.connecting_road_list_id;
    bool if_connect_1 = false;
    bool if_connect_2 = false;
    int from_id_1,from_id_2;
    int go_id_1,go_id_2;
  
    if (from_contact_point == hdmap::Road::BuildInfo::Link::kStart) {
      from_id_1 = polygons[from].left_back_id;
      from_id_2 = polygons[from].right_back_id;
      if (connecting_info.contact_point == hdmap::Road::BuildInfo::Link::kStart)
      {
        go_id_1 = polygons[go].right_back_id;
        go_id_2 = polygons[go].left_back_id;

        for (auto &lane_pair : connecting_info.lane_id_pair) {
          if (lane_pair.first==polygons[from].outline_lanes.front().first.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.front().second.lock()->list_id)
          {
            if_connect_1 = true;
          }
          if (lane_pair.first==polygons[from].outline_lanes.front().second.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.front().first.lock()->list_id)
          {
            if_connect_2 = true;
          }
        }
      }
      else
      if (connecting_info.contact_point==hdmap::Road::BuildInfo::Link::kEnd)
      {
        go_id_1 = polygons[go].left_front_id;
        go_id_2 = polygons[go].right_front_id;

        for (auto &lane_pair : connecting_info.lane_id_pair) {
          if (lane_pair.first==polygons[from].outline_lanes.front().first.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.back().first.lock()->list_id)
          {
            if_connect_1 = true;
          }
          if (lane_pair.first==polygons[from].outline_lanes.front().second.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.back().second.lock()->list_id)
          {
            if_connect_2 = true;
          }
        }
      }
      else
      {
        ROS_ERROR("[UnionRoadsideBuilder::AddEdgeJunction]: go_contact_point error1.");
        return ;
      }
      
      
    }
    else
    if (from_contact_point == hdmap::Road::BuildInfo::Link::kEnd) 
    {
      from_id_1 = polygons[from].left_front_id;
      from_id_2 = polygons[from].right_front_id;
      if (connecting_info.contact_point == hdmap::Road::BuildInfo::Link::kStart)
      {
        go_id_1 = polygons[go].left_back_id;
        go_id_2 = polygons[go].right_back_id;

        for (auto &lane_pair : connecting_info.lane_id_pair) {
          if (lane_pair.first==polygons[from].outline_lanes.back().first.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.front().first.lock()->list_id)
          {
            if_connect_1 = true;
          }
          if (lane_pair.first==polygons[from].outline_lanes.back().second.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.front().second.lock()->list_id)
          {
            if_connect_2 = true;
          }
        }
      }
      else
      if (connecting_info.contact_point==hdmap::Road::BuildInfo::Link::kEnd)
      {
        go_id_1 = polygons[go].right_front_id;
        go_id_2 = polygons[go].left_front_id;

        for (auto &lane_pair : connecting_info.lane_id_pair) {
          if (lane_pair.first==polygons[from].outline_lanes.back().first.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.back().second.lock()->list_id)
          {
            if_connect_1 = true;
          }
          if (lane_pair.first==polygons[from].outline_lanes.back().second.lock()->list_id && lane_pair.second==polygons[go].outline_lanes.back().first.lock()->list_id)
          {
            if_connect_2 = true;
          }
        }
      }
      else
      {
        ROS_ERROR("[UnionRoadsideBuilder::AddEdgeJunction]: go_contact_point error1.");
        return ;
      }
    }
    else
    {
        ROS_ERROR("[UnionRoadsideBuilder::AddEdgeJunction]: from_contact_point error.");
        return ;
    }
    
    // std::cout << "from:" << from << std::endl;
    // std::cout << "go:" << go << std::endl;
    // std::cout << "from_contact_point:" << from_contact_point << std::endl;
    // std::cout << "go_contact_point:" << connecting_info.contact_point << std::endl;
    // std::cout << "from_id_1:" << from_id_1 << std::endl;
    // std::cout << "go_id_1:" << go_id_1 << std::endl;
    // std::cout << "from_id_2:" << from_id_2 << std::endl;
    // std::cout << "go_id_2:" << go_id_2 << std::endl;
    // std::cout << "if_conenct_1:" << if_connect_1 << std::endl;
    // std::cout << "if_conenct_2:" << if_connect_2 << std::endl;
    // std::cout << "connecting_info.lane_id_pair:" << std::endl;
    // for (auto &lane_pair : connecting_info.lane_id_pair) {
    //   std::cout << lane_pair.first << " " << lane_pair.second << std::endl;
    // }
    if (if_connect_1)
    {
      (*node_list)[from_id_1].AddEdge(go_id_1,go);
      (*node_list)[go_id_1].AddEdge(from_id_1,from);
    }
    if (if_connect_2)
    {
      (*node_list)[from_id_2].AddEdge(go_id_2,go);
      (*node_list)[go_id_2].AddEdge(from_id_2,from);
    }

  }
}

bool UnionRoadsideBuilder::SegmentIsIntersectPolygon(const opendrive_common::GeometryLineSegment &check_segment,
                                                   const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list)
{
  int i, j;
  bool if_in = false;

  double distance_cached = 1e+100;
  auto last_node_id = *(polygon.node_id.rbegin());
  for (auto &now_node_id:polygon.node_id) {
    std::vector<opendrive_common::GeometryPoint> intersections;
    opendrive_common::GeometryLineSegment now_segment(node_list[last_node_id].pose,
                                    node_list[now_node_id].pose);
    if (opendrive_common::LineSegmentIsIntersectant2LineSegment(check_segment,now_segment))
      return true;
    last_node_id = now_node_id;
  }
  return false;
}

bool UnionRoadsideBuilder::FastSegmentIsIntersectPolygon(const opendrive_common::GeometryLineSegment &check_segment,
                                                   const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list)
{
  int calc_num = 0;
  double y0 = check_segment.a(1);
  double y1 = check_segment.b(1);
  if (y0 > y1) std::swap(y0, y1);

  int l = -1;
  int r = polygon.sort_edge.size()-1;
  double search_y = y0 - (1e-6);
  while (l + 1 < r) {
    int mid = (l + r) >> 1;
    if (node_list[polygon.sort_edge[mid].id[1]].pose(1) >= search_y)
      r = mid;
    else
      l = mid;
  }
  for (int check_edge = 0; check_edge < polygon.sort_edge.size();
       check_edge++) {
    //if (polygon.sort_edge[check_edge].min_y0 > y1+(1e-6)) break;
    calc_num++;
    int last_node_id = polygon.sort_edge[check_edge].id[0];
    int now_node_id = polygon.sort_edge[check_edge].id[1];
    auto now_point = node_list[now_node_id].pose;
    auto last_point = node_list[last_node_id].pose;
    opendrive_common::GeometryLineSegment now_segment(now_point,last_point);
    if (opendrive_common::LineSegmentIsIntersectant2LineSegment(check_segment,now_segment))
      return true;
  }

  return false;
}

bool UnionRoadsideBuilder::PointIsInPolygon(const opendrive_common::GeometryPoint &check_node_pose,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list) {
  int i, j;
  bool if_in = false;
  
  double distance_cached = 1e+100;
  auto last_node_id = *(polygon.node_id.rbegin());
  for (auto &now_node_id:polygon.node_id) {
    calc_num++;
    opendrive_common::GeometryLineSegment now_segment(node_list[last_node_id].pose,node_list[now_node_id].pose);
    opendrive_common::GeometryPoint p;
    double dis=opendrive_common::GetDistancePoint2LineSegment(check_node_pose,now_segment,&p);
    if (dis<0.1)  return false;
    auto now_point = node_list[now_node_id].pose;
    auto last_point = node_list[last_node_id].pose;
    if ((((now_point(1) <= check_node_pose(1)) &&
          (check_node_pose(1) < last_point(1))) ||
         ((last_point(1) <= check_node_pose(1)) &&
          (check_node_pose(1) < now_point(1)))) &&
        (check_node_pose(0) < (last_point(0) - now_point(0)) *
                            (check_node_pose(1) - now_point(1)) /
                            (last_point(1) - now_point(1)) +
                        now_point(0))) {
      if_in = !if_in;
    }
    last_node_id=now_node_id;
  }
  return if_in;
}


void UnionRoadsideBuilder::DeleteOverlapOriginalNode(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node> *node_list)
{
  std::cout << "[4/7][DeleteOverlapOriginalNode]: start"<< std::endl;
  calc_num=0;
  int delete_num = 0;
  for (int i = 0; i < polygons.size(); i++) {
    std::cout << "[4/7][DeleteOverlapOriginalNode]: " <<i << " " <<"/ " << polygons.size() << std::endl;
    auto &main_polygon = polygons[i];
    if (main_polygon.belong_junction == "-1" ) continue;
    for (int j=0;j<polygons.size();j++)
    {
      if (i==j) continue;
      auto &check_polygon = polygons[j];
      if (!main_polygon.rough_range_normal.IfIntersect(check_polygon.rough_range_normal))  continue;
      if (main_polygon.belong_junction != check_polygon.belong_junction) continue;
      std::cout<<"Checking polygon_" << i << " which size is " << polygons[i].node_id.size() << " and polygon_" << j << " which size is " << polygons[j].node_id.size() << std::endl;
      int last_main_node_id=main_polygon.node_id.back();
      bool if_last_main_node_in_polygon = FastPointIsInPolygon((*node_list)[last_main_node_id].pose,check_polygon,*node_list);
      if (if_last_main_node_in_polygon) (*node_list)[last_main_node_id].have_del=true;
      for (auto &now_main_node_id:main_polygon.node_id)
      {
        if ((*node_list)[now_main_node_id].edges.size() != 0)
        {

          union_roadside::Edge *edge;
          bool if_now_main_node_in_polygon=false;
          if (check_polygon.rough_range_normal.IfInRoughRange((*node_list)[now_main_node_id].pose))
          {
            if (FastPointIsInPolygon((*node_list)[now_main_node_id].pose,polygons[j],*node_list))
            {
              if_now_main_node_in_polygon=true;
              (*node_list)[now_main_node_id].have_del=true;
            }
          }
          int edge_id;

          if ((*node_list)[last_main_node_id].FindEdge(now_main_node_id, i,
                                                       &edge_id)) {

            if (FastSegmentIsIntersectPolygon(opendrive_common::GeometryLineSegment((*node_list)[last_main_node_id].pose,(*node_list)[now_main_node_id].pose),
                                          polygons[j],*node_list))
            {

              (*node_list)[last_main_node_id].edges[edge_id].is_intersect = true;
              int inverse_edge_id;
              if (!(*node_list)[now_main_node_id].FindEdge(last_main_node_id,i,&inverse_edge_id))
              {
                puts("why gg");
              }
              (*node_list)[now_main_node_id].edges[inverse_edge_id].is_intersect = true;
              
            } else {

              if (if_last_main_node_in_polygon && if_now_main_node_in_polygon)
              {
                RemoveUndirectedEdge(last_main_node_id, now_main_node_id,node_list);
              }
            }
          }
          if_last_main_node_in_polygon=if_now_main_node_in_polygon;
        }
        last_main_node_id = now_main_node_id;
      }
    }
    
  }
  std::cout << "[3/7][DeleteOverlapOriginalNode]: Total calculate " <<calc_num<<" times for intersections. Delete "<< delete_num << " nodes successful." << std::endl; 
}




void UnionRoadsideBuilder::DeleteOverlapIntersectionNode(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node> *node_list)
{
  
  std::cout << "[4/7][DeleteOverlapIntersectionNode]: start"<< std::endl;
  calc_num=0;
  int delete_num = 0;
  for (int i = 0; i < polygons.size(); i++) {
    std::cout << "[4/7][DeleteOverlapIntersectionNode]: " <<i << " " <<"/ " << polygons.size() << std::endl;
    auto &main_polygon = polygons[i];
    if (main_polygon.belong_junction == "-1" ) continue;
    for (int j=0;j<polygons.size();j++)
    {
      if (i==j) continue;
      auto &check_polygon = polygons[j];
      if (!main_polygon.rough_range_normal.IfIntersect(check_polygon.rough_range_normal))  continue;
      if (main_polygon.belong_junction != check_polygon.belong_junction) continue;
      std::cout<<"Checking polygon_" << i << " which size is " << polygons[i].node_id.size() << " and polygon_" << j << " which size is " << polygons[j].node_id.size() << std::endl;
      for (auto &main_node_id:main_polygon.node_id)
      {
        if ((*node_list)[main_node_id].have_del) continue;
        if (!(*node_list)[main_node_id].is_intersection_point) continue;
        if (!check_polygon.rough_range_normal.IfInRoughRange((*node_list)[main_node_id].pose)) continue;
        if (FastPointIsInPolygon((*node_list)[main_node_id].pose,polygons[j],*node_list))
        {
          (*node_list)[main_node_id].have_del=true;
          delete_num++;
        }
      }
    }
    
  }
  std::cout << "[3/7][DeleteOverlapIntersectionNode]: Total calculate " <<calc_num<<" times for intersections. Delete "<< delete_num << " nodes successful." << std::endl; 

}


void UnionRoadsideBuilder::BuildPolygonSortEdge(const std::vector<union_roadside::Node> &node_list,std::vector<union_roadside::Polygon>  *polygons)
{
  std::cout << "[3/7][BuildPolygonSortEdge]: start" << std::endl;
  for (int polygon_id = 0; polygon_id < polygons->size();polygon_id++)
  {
    (*polygons)[polygon_id].sort_edge.resize(
        (*polygons)[polygon_id].node_id.size());
    int last_node_id = (*polygons)[polygon_id].node_id.back();
    int id = 0;
    for (auto now_node_id : (*polygons)[polygon_id].node_id) 
    {
      union_roadside::SortEdgeElement sort_edge_element;
      sort_edge_element.id[0] = last_node_id;
      sort_edge_element.id[1] = now_node_id;
      if (node_list[sort_edge_element.id[0]].pose(1)>node_list[sort_edge_element.id[1]].pose(1))
      {
        std::swap(sort_edge_element.id[0], sort_edge_element.id[1]);
      }
      (*polygons)[polygon_id].sort_edge[id] = sort_edge_element;
      id++;
      last_node_id = now_node_id;
    }
    std::sort((*polygons)[polygon_id].sort_edge.begin(),
              (*polygons)[polygon_id].sort_edge.end(),
              [&](const union_roadside::SortEdgeElement &x, const union_roadside::SortEdgeElement &y) {
                if (node_list[x.id[1]].pose(1) != node_list[y.id[1]].pose(1))
                  return node_list[x.id[1]].pose(1) < node_list[y.id[1]].pose(1);
                return node_list[x.id[0]].pose(0) < node_list[y.id[0]].pose(0);
              });

    double last_min_y0=1e+100;
    
    for (int i = (*polygons)[polygon_id].sort_edge.size()-1; i >= 0; i--) {
      (*polygons)[polygon_id].sort_edge[i].min_y0 = std::min(node_list[(*polygons)[polygon_id].sort_edge[i].id[0]].pose(1),last_min_y0);
      last_min_y0 = (*polygons)[polygon_id].sort_edge[i].min_y0;
    }
  }
  std::cout << "[3/7][BuildPolygonSortEdge]: finish" << std::endl;
}

bool UnionRoadsideBuilder::FastPointIsInPolygon(const opendrive_common::GeometryPoint &check_node_pose,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list)
{

  double in_edge_dis = 1e-1;
  int l = -1;
  int r = polygon.sort_edge.size()-1;
  double search_y = check_node_pose(1) - in_edge_dis - (1e-6);
  while (l + 1 < r) {
    int mid = (l + r) >> 1;
    if (node_list[polygon.sort_edge[mid].id[1]].pose(1) >= search_y)
      r = mid;
    else
      l = mid;
  }

  int calc_num=0;
  bool if_in = false;
  for (int check_edge = r; check_edge < polygon.sort_edge.size();
       check_edge++) {
    if (polygon.sort_edge[check_edge].min_y0 > check_node_pose(1)+in_edge_dis) break;
    calc_num++;
    int last_node_id = polygon.sort_edge[check_edge].id[0];
    int now_node_id = polygon.sort_edge[check_edge].id[1];
    auto now_point = node_list[now_node_id].pose;
    auto last_point = node_list[last_node_id].pose;
    opendrive_common::GeometryLineSegment now_segment(now_point,last_point);
    opendrive_common::GeometryPoint p;
    double dis=opendrive_common::GetDistancePoint2LineSegment(check_node_pose,now_segment,&p);
    if (dis<in_edge_dis)  return false;
    if ((((now_point(1) <= check_node_pose(1)) &&
          (check_node_pose(1) < last_point(1))) ||
         ((last_point(1) <= check_node_pose(1)) &&
          (check_node_pose(1) < now_point(1)))) &&
        (check_node_pose(0) < (last_point(0) - now_point(0)) *
                            (check_node_pose(1) - now_point(1)) /
                            (last_point(1) - now_point(1)) +
                        now_point(0))) {
      if_in = !if_in;
    }
  }
  
  return if_in;
}

void UnionRoadsideBuilder::PolygonPointReduction(std::vector<opendrive_common::GeometryPolygon> *output_polygons)
{
  std::cout << "[7/7][BuildPolygonSortEdge]: start" << std::endl;
  auto &param = ParamManager::GetInstance().GetParam();
  double polygon_point_reduction_angle_limit = param.union_roadside_builder().polygon_point_reduction_angle_limit();
  std::cout << "polygon_point_reduction_angle_limit:" << polygon_point_reduction_angle_limit << std::endl;
  double cos_limit = cos(polygon_point_reduction_angle_limit/180.0*opendrive_common::kPi);
  for(int i=0;i<output_polygons->size();i++)
  {
    std::vector<opendrive_common::GeometryPoint> new_polygon((*output_polygons)[i].vertexs.size());
    int head=0,tail=-1;
    new_polygon.clear();
    // cos A =(b^2+c^2-a^2)/2bc 
    for (int j=0; j<(*output_polygons)[i].vertexs.size();j++)
    {
      auto &now_point = (*output_polygons)[i].vertexs[j];
      while (tail-head>=2)
      {

        double a = opendrive_common::GeometryLineSegment(new_polygon[tail-1],now_point).GetLength();
        double b = opendrive_common::GeometryLineSegment(new_polygon[tail],now_point).GetLength();
        double c = opendrive_common::GeometryLineSegment(new_polygon[tail-1],new_polygon[tail]).GetLength();
        double cos_a = (b*b+c*c-a*a) / (2.0*b*c);
        if (cos_a<=cos_limit)
          tail--;
        else
          break;
      }
      new_polygon[++tail] = now_point;
    }
    while (tail-head>3)
    {
      double a = opendrive_common::GeometryLineSegment(new_polygon[tail],new_polygon[head+1]).GetLength();
      double b = opendrive_common::GeometryLineSegment(new_polygon[head],new_polygon[head+1]).GetLength();
      double c = opendrive_common::GeometryLineSegment(new_polygon[head],new_polygon[tail]).GetLength();
      double cos_a = (b*b+c*c-a*a) / (2.0*b*c);
      if (cos_a<=cos_limit)
        head++;
      else
        break;
    }
    std::cout << "polygon_id: " << i << std::endl;
    std::cout << "before reduction: " << (*output_polygons)[i].vertexs.size() << std::endl;
    std::cout << "after reduction: " << (tail-head+1) << std::endl;
    (*output_polygons)[i].vertexs.clear();
    for (int j=head;j<=tail;j++)
      (*output_polygons)[i].vertexs.emplace_back(new_polygon[j]);
  }
  
  std::cout << "[7/7][BuildPolygonSortEdge]: finish" << std::endl;
}
};  // namespace roadside
};  // namespace opendrive_generator
