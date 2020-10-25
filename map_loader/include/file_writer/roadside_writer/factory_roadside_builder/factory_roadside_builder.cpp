// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "factory_roadside_builder.h"
namespace opendrive_generator {
namespace roadside {
void FactoryRoadsideBuilder::SetRoadsideBuilder(BaseRoadsideBuilder *_base_roadside_builder)
{
  base_roadside_builder_ = _base_roadside_builder;
}
void FactoryRoadsideBuilder::Build(const hdmap::EntireMap &entire_map, std::vector<opendrive_common::GeometryPolygon> *polygons)
{
  base_roadside_builder_->Build(entire_map,polygons);
}
};  // namespace roadside
};  // namespace opendrive_generator
