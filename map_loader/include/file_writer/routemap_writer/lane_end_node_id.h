// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
namespace opendrive_generator {
namespace route_map{
struct LaneEndNodeID
{
  int pos_head;
  int pos_tail;
  int neg_head;
  int neg_tail;
};
};  // namespace route
};  // namespace opendrive_generator



//   |  *1  |
//   |      |
//   |      |
//   |      |
//   |      |
//   |      |
//   |      |
//   |      |
//   |  *2  |

// 假设道路方向是从上到下，
// pos_head是*1，方向向下
// pos_tail是*2，方向向下
// neg_head是*1，方向向上
// neg_tail是*2，方向向上
