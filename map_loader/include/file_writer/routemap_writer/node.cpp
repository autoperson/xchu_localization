// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "node.h"
#include <opendrive_common.h>

namespace opendrive_generator {
    namespace route_map {
        Node::Node() {};

        Node::Node(const hdmap::LaneNodePtr &lane_node, const bool &if_reverse) {
            pose = lane_node->pose;
            if (if_reverse) pose(2) = opendrive_common::StandardAngle(pose(2) + opendrive_common::kPi);
            max_speed_limit = lane_node->max_speed_limit;
            min_speed_limit = 0;
            z = lane_node->z;
        }
    };  // namespace route_map
};  // namespace opendrive_generator
