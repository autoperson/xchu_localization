// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_MAP_MANAGER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_MAP_MANAGER_H_
#include "node.h"
#include "polygon.h"
#include <vector>

namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

struct MapManager
{
    std::vector<Polygon> polygons;
    std::vector<Node> node_list;
};
};      // namespace union_roadside
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_MAP_MANAGER_H_
