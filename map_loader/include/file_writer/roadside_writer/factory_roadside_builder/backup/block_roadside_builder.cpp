// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "block_roadside_builder.h"
#include "src/common/param_manager/param_manager.h"
#include "src/common/math_utils/math_utils_top.h"
#include <algorithm>
namespace opendrive_generator {
namespace roadside {
void BlockRoadsideBuilder::Build(const FullMap::MapManager &map_manager, std::vector<GeometryPolygon> *polygons) {
  polygons->clear();
  if (map_manager.lane_list_.lanes.size() == 0) return;
  GetRoughRange(map_manager, &rough_range_normal_);
  BuildByBlock(map_manager, rough_range_normal_, polygons);
}

void BlockRoadsideBuilder::GetRoughRange(const FullMap::MapManager &map_manager,Lane::RoughRange *rough_range_normal)
{
  double max_x = -1e+100;
  double max_y = -1e+100;
  double min_x = +1e+100;
  double min_y = +1e+100;
  for (auto &lane : map_manager.lane_list_.lanes)
  {
    max_x = std::max(max_x, lane.rough_range_normal.max_x);
    max_y = std::max(max_y, lane.rough_range_normal.max_y);
    min_x = std::min(min_x, lane.rough_range_normal.min_x);
    min_y = std::min(min_y, lane.rough_range_normal.min_y);
  }
  max_x+=1e-2;
  max_y+=1e-2;
  rough_range_normal->setData(max_x,min_x,max_y,min_y);
}

void BlockRoadsideBuilder::BuildByBlock(const FullMap::MapManager &map_manager,const Lane::RoughRange &rough_range_normal,std::vector<GeometryPolygon> *polygons)
{
  const OpendriveGeneratorParam::Param &param = ParamManager::GetInstance().GetGeneratorParam();
  double ceil_length = param.union_roadside_builder().min_segment_length();
  int height= Compare::PosCeil(normal_rough_range.max_y-normal_rough_range.min_y,1e-6);
  int width = Compare::PosCeil(normal_rough_range.max_x-normal_rough_range.min_x,1e-6);
  
  Init(height,width);
  for (int i=0;i<height;i++)
    for (int j=0;j<width;j++)
    {
      auto &now_block=blocks_[i][j];
      double min_x = i*ceil_length + normal_rough_range_.min_x;
      double max_x = std::min(min_x + ceil_length, rough_range_normal.max_x);
      double min_y = j*ceil_length + normal_rough_range_.min_y;
      double max_y = std::min(min_y + ceil_length, rough_range_normal.max_y);
      now_block.rough_range_normal.SetData(max_x,min_x,max_y,min_y);
      BuildSingleBlock(map_manager,&now_block);
    }
}

void BlockRoadsideBuilder::Init(const int &height,const int &width)
{
  blocks_.resize(height);
  for (auto &row_block:blocks_)
  {
    row_block.resize(width);
    for (auto &block:row_block)
    {
      block.Clear();
    }
  }
  outlines_.clear();
  road_id2semi_outline_id_.clear();
}

void BlockRoadsideBuilder::BuildSingleBlock(const FullMap::MapManager &map_manager,Roadside::Block *now_block)
{
  GeometryPolygon block_outline;
  now_block.rough_range_normal.Transform2Polygon(&block_outline); 
  semi_outlines_.clear();
  for (auto &road:map_manager.road_list_.roads)
  {
    BuildSemiOutline(map_manager,road,now_block,&semi_outlines_);
    
  }
}

void BlockRoadsideBuilder::BuildSemiOutline(const FullMap::MapManager &map_manager,const Road &road, Roadside::Block *now_block)

{
  static std::vector<std::pair<int,int>> outline_lane_id; //left,right;
  BuildOutlineLaneID(map_manager,road,&outline_lane_id);
  static CheckSequence check_sequence;
  BuildCheckSequence(map_manager,road,outline_lane_id,&check_sequence);
  BuildBreakID(map_manager,road,now_block,&check_sequence);
  CutCheckSequence(map_manager,road,check_sequence,now_block);
  
}

void BlockRoadsideBuilder::BuildOutlineLaneID(const FullMap::MapManager &map_manager,const Road &road,std::vector<std::pair<int,int>> *outline_lane_id)
{
  outline_lane_id->clear();
  std::vector<int> sorted_lane_id(road.lane_ids);
  std::sort(sorted_lane_id.begin(),sorted_lane_id.end(),[&](const int &x, const int &y) {
              auto &x_full_id = map_manager.lane_list.lanes[x].full_id;
              auto &y_full_id = map_manager.lane_list.lanes[y].full_id;
              if (x_full_id.lane_section_id!=y_full_id.lane_section_id) return x_full_id.lane_section_id>y_full_id.lane_section_id;
              return x_full_id.lane_id!=y_full_id.lane_id;
            }); 
                        
  int left=-1,right=-1;
  int check_point_sequence_num=0;
  for (int i=0;i<sorted_lane_id.size();i++)
  {
    if (i==0|| map_manager.lane_list.lanes[sorted_lane_id[i]].full_id.lane_section_id
             !=map_manager.lane_list.lanes[sorted_lane_id[i-1]].full_id.lane_section_id)
    {
      left=i;
    }
    if (i==sorted_lane_id.size()-1 || map_manager.lane_list.lanes[sorted_lane_id[i]].full_id.lane_section_id
             !=map_manager.lane_list.lanes[sorted_lane_id[i+1]].full_id.lane_section_id)
    {
      right=i;
      outline_lane_id->emplace_back(std::make_pair(left,right));
    }
  }
}

void BlockRoadsideBuilder::BuildCheckSequence(const FullMap::MapManager &map_manager,const Road &road,const std::vector<std::pair<int,int>> &outline_lane_id,CheckSequence *check_sequence)
{
  check_sequence->Clear();
  int check_sequence_point_num=0;
  for (int i=0;i<outline_lane_id.size();i++)
  {
    check_sequence_point_num+=map_manager.lane_list.lanes[outline_lane_id[i].first].left_line_ids.size();
    check_sequence_point_num+=map_manager.lane_list.lanes[outline_lane_id[i].second].right_line_ids.size();
  }
  check_sequence->ids.resize(check_sequence_point_num);
  int now_num=0;
  check_sequence.left_end_id=0;
  for (int i=outline_lane_id.size()-1;i>=0;i--)
  {
    for (auto &node_id:map_manager.lane_list.lanes[outline_lane_id[i].first].left_line_id)
      check_sequence->ids[now_num++] = node_id;  
  }
  check_sequence.right_end_id=now_num-1;
  for (int i=0;i<outline_lane_id.size();i++)
  {
    for (auto &node_id:map_manager.lane_list.lanes[outline_lane_id[i].second].right_line_id)
      check_sequence->ids[now_num++] = node_id;  
  }

  
  
  
  
}

void BlockRoadsideBuilder::BuildBreakID(const FullMap::MapManager &map_manager,const Road &road,const RoadSide::Block &now_block, const std::vector<std::pair<int,int>> &outline_lane_id,CheckSequence *check_sequence) 
{
  if (road.build_info.contact_point==Road::BuildInfo::Link::CONTACT_POINT_UNKNOWN)
  {
    check_sequence->break_id = check_sequence->left_end_id;  
    return ;
  }
  else
  if (!now_block.rough_range_normal.IfInRoughRange(check_sequence.ids[check_sequence.left_end_id]))
  {
    check_sequence->break_id = check_sequence->left_end_id;  
    return ;
  }
  else
  {
    int now=check_sequence->left_end_id;
    int last=now-1;
    if (last<0) last=check_sequence->ids.size()-1;
    for (int i=0;i<check_sequence->ids.size();i++)
    {
      GeometryPoint now_point=map_manager.node_list.nodes[ check_sequence->ids[now] ].pose;
      GeometryPoint last_point=map_manager.node_list.nodes[ check_sequence->ids[last] ].pose;
      if (now_block.rough_range_normal.IfIntersectBoundary(GeometryLineSegment(now_point,last_point)));
      {
        check_sequence->break_id = now;  
        return ;
      }
      now=last--;
      if (last<0) last=check_sequence->ids.size()-1;
    }
  }
}

void BlockRoadsideBuilder::CutCheckSequence(const FullMap::MapManager &map_manager,const Road &road,const CheckSequence check_sequence,const RoadSide::Block &now_block)
{

}
};
};