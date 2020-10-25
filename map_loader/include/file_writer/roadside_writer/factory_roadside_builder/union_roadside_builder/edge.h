// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_EDGE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_EDGE_H_

namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

struct Edge {
  int to_node_id;
  int belong_polygon_id;
  bool is_intersect;
  Edge() {}
  Edge(const int &_to_node_id, const int &_belong_polygon_id)
      : to_node_id(_to_node_id), belong_polygon_id(_belong_polygon_id) {}
};

};  // namespace union_roadside
};  // namespace roadside
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_EDGE_H_