// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROUTEMAP_WRITER_ROUTEMAP_WRITER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROUTEMAP_WRITER_ROUTEMAP_WRITER_H_
#include <string>
#include <hdmap.h>
#include "node.h"
#include "lane_end_node_id.h"
#include <vector>

namespace opendrive_generator
{

// ToDo: 有可能会出现重合点的情况，应该先merge一下重合的点，然后再输出
class RoutemapWriter {
 public:
  void Write(const std::string &file_path, const hdmap::EntireMap &entire_map);
  static RoutemapWriter &GetInstance(void) {
    static RoutemapWriter this_singleton;
    return this_singleton;
  }
 private:
  void Clear();
  void BuildNodeInLane(const hdmap::EntireMap &entire_map);
  void BuildNodeBetweenLane(const hdmap::EntireMap &entire_map);
  void AddEdge(const int &from, const int &to);
  void WriteFiles(const std::string &file_path);
  std::vector<route_map::Node> node_list_;
  std::vector<route_map::LaneEndNodeID> lane_end_node_ids_;
  std::set<std::pair<int, int>> edge_list_;
};

};  // namespace RoutemapNS

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROUTEMAP_WRITER_ROUTEMAP_WRITER_H_
