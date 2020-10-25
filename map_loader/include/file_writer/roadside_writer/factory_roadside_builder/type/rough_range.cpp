// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "rough_range.h"
namespace opendrive_generator 
{
namespace roadside
{
//  1----2
//  |    |
//  |    |
//  0----3
// 0 = (min_x,min_y)
// 1 = (min_x,max_y)
// 2 = (max_x,max_y)
// 3 = (max_x,min_y)
//平面直角坐标系
void RoughRange::SetData(const double &_max_x,const double &_min_x,const double &_max_y,const double &_min_y)
{
  max_x=_max_x;
  min_x=_min_x;
  max_y=_max_y;
  min_y=_min_y;

  polygon.SetVertexAmount(4);
  polygon.vertexs[0](0) = min_x; 
  polygon.vertexs[0](1) = min_y;

  polygon.vertexs[1](0) = min_x; 
  polygon.vertexs[1](1) = max_y;

  polygon.vertexs[2](0) = max_x; 
  polygon.vertexs[2](1) = max_y;

  polygon.vertexs[3](0) = max_x; 
  polygon.vertexs[3](1) = min_y;
}
 
bool RoughRange::IfInRoughRange(const opendrive_common::GeometryPoint &p) const
{
  return min_x<=p(0) && p(0)<=max_x && min_y<=p(1) && p(1)<=max_y;
}

bool RoughRange::IfIntersectBoundary(const opendrive_common::GeometryLineSegment &seg) const
{
  return opendrive_common::LineSegmentIsIntersectant2Polygon(seg,polygon);
}

bool RoughRange::IfIntersect(const RoughRange &rough_range) const
{
  if (max_x<rough_range.min_x) return false;
  if (rough_range.max_x<min_x) return false;
  if (max_y<rough_range.min_y) return false;
  if (rough_range.max_y<min_y) return false;
  return true;
}

};
};