// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_NODE_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_NODE_H_
#include <Eigen/Dense>
#include "edge.h"
#include <vector>
namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

struct Node
{
    Eigen::Vector3d pose;
    bool have_del;
    bool have_in_stack;
    bool is_intersection_point;
    std::vector<union_roadside::Edge> edges;
    std::vector<int> belong_polygon_id;
    
    void Init(const Eigen::Vector3d _pose,const int _polygon_id); //普通点
    void Init(const Eigen::Vector3d _pose,const int _polygon_id_1,const int _polygon_id_2); // 交点
    bool IfBelong(const int &polygon_id);
    void RemoveEdge(const int &node_id);
    void AddEdge(const int _to_node_id,const int _belong_polygon);
    bool FindEdge(const int _to_node_id,const int _belong_polygon,int * edge_id);
    bool HaveEdge(const int _to_node_id,const int _belong_polygon);
    void Clear();
};

};  // namespace union_roadside
};  // namespace roadside
};  // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_FACTORY_ROADSIDE_BUILDER_UNION_ROADSIDE_BUILDER_NODE_H_
