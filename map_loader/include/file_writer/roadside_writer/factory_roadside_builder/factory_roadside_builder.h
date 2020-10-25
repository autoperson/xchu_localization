// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_FACTORY_ROADSIDE_BUILDER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_FACTORY_ROADSIDE_BUILDER_H_
#include <Eigen/Dense>
#include <string>
#include "roadside_builder_top.h"
namespace opendrive_generator {
namespace roadside {
class FactoryRoadsideBuilder
{
 public:
  void SetRoadsideBuilder(BaseRoadsideBuilder *_base_roadside_builder);
  void Build(const hdmap::EntireMap &entire_map, std::vector<opendrive_common::GeometryPolygon> *polygons);
 private:
  BaseRoadsideBuilder *base_roadside_builder_;
};
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_FACTORY_ROADSIDE_BUILDER_H_
