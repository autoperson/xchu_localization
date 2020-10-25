// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_H_
#include "base_roadside_builder.h"
#include "union_roadside_builder/union_roadside_type.h"
#include "param_manager/param_manager.h"

#include <vector>
#include <stack>
#include <opendrive_common.h>
namespace opendrive_generator {
namespace roadside {
class UnionRoadsideBuilder : public BaseRoadsideBuilder
{
 public:
  virtual void Build(const hdmap::EntireMap &entire_map, std::vector<opendrive_common::GeometryPolygon> *output_polygons);
  static UnionRoadsideBuilder &GetInstance(void) {
    static UnionRoadsideBuilder this_singleton;
    return this_singleton;
  }
  

 private:
  bool debug_mode = false;
  std::string base_debug_path = "/home/udi/opendrive_ws/debug_info/";
  std::vector<union_roadside::Polygon> polygons_;
  std::vector<union_roadside::Node> node_list_;
  void BuildOriginPolygon(const hdmap::EntireMap &entire_map, std::vector<union_roadside::Polygon>  *polygons,std::vector<union_roadside::Node>  *node_list);
  void BuildOutlineLaneID(const hdmap::RoadPtr &road,std::vector<std::pair<std::weak_ptr<hdmap::Lane>, std::weak_ptr<hdmap::Lane>>> *outline_lanes);
  void BuildConnection(const hdmap::EntireMap &entire_map,const std::vector<union_roadside::Polygon>  &polygon,std::vector<union_roadside::Node>  *node_list);
  //void AddEdgeDifferentRoad(const std::vector<UnionRoadside::Polygon>  &polygon,const int &from,const int &go,const FullMap::Road::BuildInfo::Link::ContactPoint &from_contact_point,const FullMap::Road::BuildInfo::Link::ContactPoint &go_contact_point,const ConnectionInfo &conection_info,std::vector<UnionRoadside::Node>  *node_list);
  //void AddEdgeDifferentRoad(const std::vector<UnionRoadside::Polygon>  &polygon,const int &from, const int &from_node_id_1, const int &from_node_id_2, const FullMap::RoadEdge &road_edge, std::vector<UnionRoadside::Node>  *node_list);
  
  //很奇怪的是，有的lane的连接关系，在junction里面没有连接，但是在Road标签里面的lane标签里会有连接，所以都要试着连一下
  void AddEdgeJunction(const std::vector<union_roadside::Polygon>  &polygon,const int &from,const std::list<ConnectionInfo> &list_connecting_info,const hdmap::Road::BuildInfo::Link::ContactPoint &from_contact_point,std::vector<union_roadside::Node>  *node_list);
  void BuildPolygonRoughInfo(const std::vector<union_roadside::Node> &node_list,std::vector<union_roadside::Polygon>  *polygons);
  void BuildPolygonSortEdge(const std::vector<union_roadside::Node> &node_list,std::vector<union_roadside::Polygon>  *polygons);

  void InsertIntersectPoint(std::vector<union_roadside::Polygon>  *polygons,std::vector<union_roadside::Node>  *node_list);
  void DeleteOverlapNode(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node>  *node_list);
  void DeleteOverlapOriginalNode(const std::vector<union_roadside::Polygon> &polygons, std::vector<union_roadside::Node> *node_list_);
  void DeleteOverlapIntersectionNode(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node> *node_list);
  bool PointIsInPolygon(const opendrive_common::GeometryPoint &check_node_pose,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list);
  bool FastPointIsInPolygon(const opendrive_common::GeometryPoint &check_node_pose,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list);
  bool SegmentIsIntersectPolygon(const opendrive_common::GeometryLineSegment &segment,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list);                                        
  bool FastSegmentIsIntersectPolygon(const opendrive_common::GeometryLineSegment &check_segment,
                                            const union_roadside::Polygon &polygon,const std::vector<union_roadside::Node> &node_list);  
  void DeleteOverlapEdge(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node>  *node_list);
  void BuildOutputPolygon(const std::vector<union_roadside::Polygon> &polygons,std::vector<union_roadside::Node> *node_list,std::vector<opendrive_common::GeometryPolygon> *output_polygons);
  void BuildSingleRoad(const hdmap::RoadPtr &road, const std::vector< std::pair<std::weak_ptr<hdmap::Lane>, std::weak_ptr<hdmap::Lane> > >  &outline_lanes, const int &polygon_id,union_roadside::Polygon *polygon,std::vector<union_roadside::Node>  *node_list);
  

  void RemoveUndirectedEdge(const int &x,const int &y,std::vector<union_roadside::Node> *node_list);
  void AddUndirectedEdge(const int &x,const int &y,const int &polygon_id,std::vector<union_roadside::Node> *node_list);
  void PolygonPointReduction(std::vector<opendrive_common::GeometryPolygon> *output_polygons);
  //void OutputDebugInfo(const std::string &file_path,const std::vector<UnionRoadside::Polygon> polygons,const std::vector<UnionRoadside::Node> &node_list);
};
};  // namespace roadside
};  // namespace opendrive_generator  
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_H_
