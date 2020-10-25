// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_SORT_EDGE_ELEMENT_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_SORT_EDGE_ELEMENT_H_
namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

struct SortEdgeElement  
{
  int id[2];  // 保证 id[0]的node的y <= id[1]的node的y
  double min_y0; // node_list排在该元素后面的所有edge中,id[0]中的最小值
};
};      // namespace union_roadside
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_SORT_EDGE_ELEMENT_H_
