// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "routemap_writer.h"
#include "geometry_msgs/Pose.h"
#include <opendrive_common.h>

namespace opendrive_generator {
    void RoutemapWriter::Write(const std::string &file_path, const hdmap::EntireMap &entire_map) {
        Clear();
        BuildNodeInLane(entire_map);
        BuildNodeBetweenLane(entire_map);
        WriteFiles(file_path);
    }

    void RoutemapWriter::Clear() {
        node_list_.clear();
        lane_end_node_ids_.clear();
        edge_list_.clear();
    }

    void RoutemapWriter::BuildNodeInLane(const hdmap::EntireMap &entire_map) {
        auto &lane_list = entire_map.lane_list();
        for (auto &lane:lane_list) {

            route_map::LaneEndNodeID lane_end_node_id;
            lane_end_node_id.pos_head = lane_end_node_id.pos_tail = lane_end_node_id.neg_head = lane_end_node_id.neg_tail = -1;
            if (lane->type == hdmap::Lane::kDriving) {
                if (lane->CanGoForward()) {
                    if (lane->lane_nodes.size() == 0) {
                        opendrive_common::Output::Error(
                                "[RoutemapWriter::BuildNodeInLane]1:lane.lane_node_ids.size()==0");
                    }
                    lane_end_node_id.pos_head = node_list_.size();

                    for (auto &lane_node : lane->lane_nodes) {
                        node_list_.emplace_back(route_map::Node(lane_node, false));
                    }
                    lane_end_node_id.pos_tail = node_list_.size() - 1;
                    for (int i = lane_end_node_id.pos_head + 1; i <= lane_end_node_id.pos_tail; i++)
                        AddEdge(i - 1, i);
                }

                if (lane->CanGoBackward()) {
                    if (lane->lane_nodes.size() == 0) {
                        opendrive_common::Output::Error(
                                "[RoutemapWriter::BuildNodeInLane]2:lane.lane_node_ids.size()==0");
                    }
                    lane_end_node_id.neg_head = node_list_.size();
                    for (auto &lane_node : lane->lane_nodes) {
                        node_list_.emplace_back(route_map::Node(lane_node, true));
                    }
                    lane_end_node_id.neg_tail = node_list_.size() - 1;
                    for (int i = lane_end_node_id.neg_tail; i >= lane_end_node_id.neg_head + 1; i--)
                        AddEdge(i, i - 1);
                }
            }
            lane_end_node_ids_.emplace_back(lane_end_node_id);
        }
    }

    void RoutemapWriter::BuildNodeBetweenLane(const hdmap::EntireMap &entire_map) {
        auto &lane_list = entire_map.lane_list();

        for (int lane_list_id = 0; lane_list_id < lane_list.size();
             lane_list_id++) {
            auto &lane = lane_list[lane_list_id];
            auto &lane_end_node_id = lane_end_node_ids_[lane_list_id];
            for (auto &lane_edge:lane->predecessor_lane_edges) {
                int now_id;
                int next_id;
                int connecting_lane_list_id = lane_edge.connecting_lane.lock()->list_id;
                if (lane_edge.contact_point == hdmap::LaneEdge::kStart) {
                    now_id = lane_end_node_ids_[connecting_lane_list_id].neg_head;
                    next_id = lane_end_node_id.pos_head;
                    AddEdge(now_id, next_id);
                    now_id = lane_end_node_id.neg_head;
                    next_id = lane_end_node_ids_[connecting_lane_list_id].pos_head;
                    AddEdge(now_id, next_id);
                } else if (lane_edge.contact_point == hdmap::LaneEdge::kEnd) {
                    now_id = lane_end_node_ids_[connecting_lane_list_id].pos_tail;
                    next_id = lane_end_node_id.pos_head;
                    AddEdge(now_id, next_id);
                    now_id = lane_end_node_id.neg_head;
                    next_id = lane_end_node_ids_[connecting_lane_list_id].neg_tail;
                    AddEdge(now_id, next_id);
                } else {
                    opendrive_common::Output::Error("[RoutemapWriter::BuildNodeBetweenLane]1:LaneEdge type error");
                }
            }

            for (auto &lane_edge:lane->successor_lane_edges) {
                int now_id;
                int next_id;
                int connecting_lane_list_id = lane_edge.connecting_lane.lock()->list_id;
                if (lane_edge.contact_point == hdmap::LaneEdge::kStart) {
                    now_id = lane_end_node_id.pos_tail;
                    next_id = lane_end_node_ids_[connecting_lane_list_id].pos_head;
                    AddEdge(now_id, next_id);
                    now_id = lane_end_node_ids_[connecting_lane_list_id].neg_head;
                    next_id = lane_end_node_id.neg_tail;
                    AddEdge(now_id, next_id);
                } else if (lane_edge.contact_point == hdmap::LaneEdge::kEnd) {
                    now_id = lane_end_node_id.pos_tail;
                    next_id = lane_end_node_ids_[connecting_lane_list_id].neg_tail;
                    AddEdge(now_id, next_id);
                    now_id = lane_end_node_ids_[connecting_lane_list_id].pos_tail;
                    next_id = lane_end_node_id.neg_tail;
                    AddEdge(now_id, next_id);
                } else {
                    opendrive_common::Output::Error("[RoutemapWriter::BuildNodeBetweenLane]2:LaneEdge type error");
                }
            }
        }
    }

    void RoutemapWriter::AddEdge(const int &from, const int &to) {
        if (from == -1 || to == -1) return;
        edge_list_.emplace(std::make_pair(from, to));
    }

    void RoutemapWriter::WriteFiles(const std::string &file_path) {
        FILE *fpt = fopen(file_path.c_str(), "w");

        for (int i = 0; i < node_list_.size(); i++) {
            geometry_msgs::Pose pose;
            opendrive_common::VectorXYYaw2GeometryPose(node_list_[i].pose, &pose);
            pose.position.z = node_list_[i].z;
            fprintf(fpt, "VERTEX_SE3:QUAT %d %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
                    i,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                    node_list_[i].min_speed_limit, node_list_[i].max_speed_limit);

        }

        for (auto &edge:edge_list_) {
            fprintf(fpt,
                    "EDGE_SE3:QUAT %d %d "
                    "0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 0 0 1 \n",
                    edge.first, edge.second);
        }

        fclose(fpt);
    }
};