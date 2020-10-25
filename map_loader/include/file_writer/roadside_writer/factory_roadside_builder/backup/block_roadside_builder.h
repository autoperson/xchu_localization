// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BLOCK_ROADSIDE_BUILDER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BLOCK_ROADSIDE_BUILDER_H_
#include "base_roadside_builder.h"
#include "type/roadside_type_top.h"
namespace opendrive_generator {
namespace roadside {
class BlockRoadsideBuilder : public BaseRoadsideBuilder
{
 public:
  virtual void Build(const FullMap::MapManager &map_manager, std::vector<GeometryPolygon> *polygons);
  static BlockRoadsideBuilder &GetInstance(void) {
    static BlockRoadsideBuilder this_singleton;
    return this_singleton;
  }
  

 private:
  Roadside::RoughRange rough_range_normal_;
  std::vector<std::vector<Roadside::Block> > blocks_;
  std::vector<std::list<Roadside::Node> > outlines_;
  std::map<std::string,std::vector<int> > road_id2semi_outline_id_;
  std::vector<Roadside::SemiOutline> semi_outlines_;
  void GetRoughRange(const FullMap::MapManager &map_manager, Lane::RoughRange *normal_rough_range);
  void BuildByBlock(const FullMap::MapManager &map_manager, const Lane::RoughRange &normal_rough_range, std::vector<GeometryPolygon> *polygons);
  void BuildSingleBlock(const FullMap::MapManager &map_manager,Roadside::Block *now_block);
  void Init(const int &height,const int &width);
};
};      // namespace roadside
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_BLOCK_ROADSIDE_BUILDER_H_
