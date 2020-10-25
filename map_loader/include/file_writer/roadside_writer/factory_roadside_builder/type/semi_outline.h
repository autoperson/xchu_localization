// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_SEMI_OUTLINE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_SEMI_OUTLINE_H_
#include <vector>

#include "node.h"
namespace opendrive_generator {
namespace roadside {
struct SemiOutline {
  std::vector<roadside::Node> nodes;
  enum EndPointType { END_POINT_UNKNOWN, LEFT, RIGHT } head, tail;
};
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_SEMI_OUTLINE_H_
