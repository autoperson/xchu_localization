// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_NODE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_NODE_H_
#include <Eigen/Dense>
namespace opendrive_generator {
namespace roadside {

struct Node {
  Eigen::Vector3d pose;
};

};  // namespace roadside
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_NODE_H_
