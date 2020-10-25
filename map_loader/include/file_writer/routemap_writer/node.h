// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
#include <Eigen/Dense>
#include <hdmap.h>
#include <opendrive_common.h>
namespace  opendrive_generator {
namespace route_map {
struct Node {
  Eigen::Vector3d pose;
  double z;
  double max_speed_limit;
  double min_speed_limit;
  Node();
  Node(const hdmap::LaneNodePtr &lane_node, const bool &if_reverse);
};
};  // namespace  route_map
};  // namespace opendrive_generator
