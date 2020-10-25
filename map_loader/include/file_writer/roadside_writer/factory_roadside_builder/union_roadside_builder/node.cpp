// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "node.h"
#include <iostream>
#include "ros/ros.h"
namespace opendrive_generator {
namespace roadside {
namespace union_roadside {

void Node::RemoveEdge(const int &node_id)
{
    for (auto iter=edges.begin();iter!=edges.end();iter++)
    {
        if (iter->to_node_id==node_id)
        {
            edges.erase(iter);
            return ;
        }
    }
}

void Node::Init(const Eigen::Vector3d _pose,const int _polygon_id)
{
    Clear();
    pose=_pose;
    belong_polygon_id.emplace_back(_polygon_id);
}

void Node::Init(const Eigen::Vector3d _pose,const int _polygon_id_1,const int _polygon_id_2)
{
    Clear();
    pose=_pose;
    belong_polygon_id.resize(2);
    belong_polygon_id[0]=_polygon_id_1;
    belong_polygon_id[1]=_polygon_id_2;
}
bool Node::IfBelong(const int &check_polygon_id)
{
    for (auto &polygon_id:belong_polygon_id)
    {
        if (polygon_id==check_polygon_id) return true;
    }
    return false;
}

void Node::Clear()
{
  have_in_stack = false;
  have_del = false;
  is_intersection_point = false;
  edges.clear();
  belong_polygon_id.clear();
}
void Node::AddEdge(const int _to_node_id,const int _belong_polygon)
{
    for (auto &edge:edges)
    {
        if (_to_node_id==edge.to_node_id)
        {
            // if (_belong_polygon!=edge.belong_polygon_id)
            // {
            //     ROS_ERROR("the same edge in different polygon");
            //     std::cout << "to_node_id = " <<_to_node_id << std::endl;
            //     std::cout << "belong_polygon = " << _belong_polygon << " and "
            //               << edge.belong_polygon_id << std::endl;
            //     std::cout << "pose= (" << pose(0) << ", " << pose(1) << ")"
            //               << std::endl;
            //     std::cout << "pause" << std::endl;
            //     getchar();
            // }
            return ;
        }
    }
    edges.emplace_back(Edge(_to_node_id, _belong_polygon));
}
bool Node::FindEdge(const int _to_node_id,const int _belong_polygon,int * edge_id)
{
    for (int i=0;i<edges.size();i++)
    {
        if (_to_node_id==edges[i].to_node_id && _belong_polygon==edges[i].belong_polygon_id)
        {
          *edge_id = i;
          return true;
        }
    }
    return false;
}

bool Node::HaveEdge(const int _to_node_id,const int _belong_polygon)
{
    for (int i=0;i<edges.size();i++)
    {
        if (_to_node_id==edges[i].to_node_id && _belong_polygon==edges[i].belong_polygon_id)
        {
            return true;
        }
    }
    return false;
}
};  // namespace union_roadside
};  // namespace roadside
};  // namespace opendrive_generator

