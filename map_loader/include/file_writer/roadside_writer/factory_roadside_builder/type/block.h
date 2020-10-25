// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_BLOCK_H_  
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_BLOCK_H_  
#include "rough_range.h"
#include "semi_outline.h"
#include <vector>
namespace opendrive_generator 
{
namespace roadside
{
struct Block
{
    Roadside::RoughRange rough_range_normal;
    std::vector<roadside::SemiOutline> semi_outline;
};
};  // namespace roadside
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_TYPE_BLOCK_H_  
