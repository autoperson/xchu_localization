// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CONNECTION_INFO_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CONNECTION_INFO_H_
#include <list>
#include <utility>
#include <opendrive_common.h>
namespace opendrive_generator 
{
namespace roadside
{
struct ConnectionInfo
{
  int connecting_road_list_id;
  hdmap::Road::BuildInfo::Link::ContactPoint contact_point;
  std::list<std::pair<int, int> > lane_id_pair;
};
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_CONNECTION_INFO_H_
