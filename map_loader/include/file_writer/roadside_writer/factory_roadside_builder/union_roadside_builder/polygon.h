// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_POLYGON_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_POLYGON_H_
#include <vector>
#include "node.h"
#include "sort_edge_element.h"

#include <opendrive_common.h>
#include "../type/roadside_type_top.h"
namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

struct Polygon
{
  std::list<int> node_id;
  roadside::RoughRange rough_range_normal;
  std::vector<SortEdgeElement> sort_edge;
  std::vector<std::pair<std::weak_ptr<hdmap::Lane>, std::weak_ptr<hdmap::Lane>>> outline_lanes;
  std::string belong_junction;
  int left_front_id;
  int right_front_id;
  int left_back_id;
  int right_back_id;
};
};      // namespace union_roadside
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_POLYGON_H_
