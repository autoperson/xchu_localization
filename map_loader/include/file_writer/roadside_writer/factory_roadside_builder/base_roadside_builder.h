// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BASE_ROADSIDE_BUILDER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BASE_ROADSIDE_BUILDER_H_
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <opendrive_common.h>
#include <vector>
#include <hdmap.h>
namespace opendrive_generator {
namespace roadside {
class BaseRoadsideBuilder
{
 public:
  virtual void Build(const hdmap::EntireMap &entier_map, std::vector<opendrive_common::GeometryPolygon> *polygons) = 0;
};
};  // namespace roadside
};  // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BASE_ROADSIDE_BUILDER_H_
