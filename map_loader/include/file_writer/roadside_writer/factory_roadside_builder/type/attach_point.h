// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ATTACH_POINT_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ATTACH_POINT_H_
namespace opendrive_generator 
{
namespace roadside
{
struct AttachPoint
{
  int list_id;
  enum Type{TYPE_UNKNOWN,HEAD,TAIL} type;
};
};  // namespace roadside
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_ATTACH_POINT_H_
