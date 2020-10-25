// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CHECK_SEQUENCE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CHECK_SEQUENCE_H_
#include <vector>
namespace opendrive_generator 
{
namespace roadside
{
struct CheckSequence
{
  std::vector<int> ids;
  int left_end_id; // 左边车道线的最后的节点
  int right_end_id; // 右边车道线的最前的节点
  int break_id;
  CheckSequence(){Clear();}
  void Clear()
  {
      ids.clear();
      left_end_id=-1;
      right_end_id=-1;
      break_id=-1;
  }
};
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CHECK_SEQUENCE_H_