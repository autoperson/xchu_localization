// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ROUGH_RANGE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ROUGH_RANGE_H_
#include <opendrive_common.h>
namespace opendrive_generator {
namespace roadside {
struct RoughRange {
  double max_x;
  double min_x;
  double max_y;
  double min_y;
  opendrive_common::GeometryPolygon polygon;
  void SetData(const double &_max_x, const double &_min_x, const double &_max_y,
               const double &_min_y);
  bool IfInRoughRange(const opendrive_common::GeometryPoint &p) const;
  bool IfIntersect(const RoughRange &check_rough_range) const;
  bool IfIntersectBoundary(
      const opendrive_common::GeometryLineSegment &segment) const;
};
};  // namespace roadside
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ROUGH_RANGE_H_
