// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
#include "debug_publisher.h"
#include "visualization_msgs/Marker.h"

namespace opendrive_generator {
    void DebugPublisher::InitRos(ros::NodeHandle *nh) { nh_ = nh; }

    void DebugPublisher::PublishPolygon(const std::string &topic_name,
                                        const opendrive_common::GeometryPolygon &polygon) {
        static ros::Publisher pub_;
        pub_ = nh_->advertise<visualization_msgs::Marker>(topic_name.c_str(), 10);

        static visualization_msgs::Marker marker;
        marker.header.frame_id = "map_tmk_opendrive";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Polygon";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1;
        marker.points.clear();

        for (int i = polygon.vertexs.size() - 1, j = 0; j < polygon.vertexs.size();
             i = j++) {
            static geometry_msgs::Point now_point;
            static geometry_msgs::Point next_point;
            now_point.x = polygon.vertexs[i](0);
            now_point.y = polygon.vertexs[i](1);
            now_point.z = 0.02;
            next_point.x = polygon.vertexs[j](0);
            next_point.y = polygon.vertexs[j](1);
            next_point.z = 0.02;
            marker.points.emplace_back(now_point);
            marker.points.emplace_back(next_point);
        }
        ros::Rate loop_rate(10);
        for (int i = 1; i <= 10; i++) {
            if (ros::ok()) {
                pub_.publish(marker);
                ros::spinOnce();
                ROS_INFO("ok");
                loop_rate.sleep();
            } else {
                ROS_ERROR("ros not ok");
            }
        }

    }


    void DebugPublisher::PublishPoints(const std::string &topic_name,
                                       const std::vector <opendrive_common::GeometryPoint> &points) {
        static ros::Publisher pub_;
        pub_ = nh_->advertise<visualization_msgs::Marker>(topic_name.c_str(), 10);

        static visualization_msgs::Marker marker;
        marker.header.frame_id = "map_tmk_opendrive";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Polygon";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 1;
        marker.points.clear();

        for (auto &point :points) {
            static geometry_msgs::Point now_point;
            now_point.x = point(0);
            now_point.y = point(1);
            now_point.z = 0.02;
            marker.points.emplace_back(now_point);
        }
        ros::Rate loop_rate(10);
        for (int i = 1; i <= 10; i++) {
            if (ros::ok()) {
                pub_.publish(marker);
                ros::spinOnce();
                ROS_INFO("ok");
                loop_rate.sleep();
            } else {
                ROS_ERROR("ros not ok");
            }
        }

    }

    void DebugPublisher::PublishEdges(const std::string &topic_name,
                                      const std::vector <opendrive_common::GeometryPoint> &points) {
        static ros::Publisher pub_;
        pub_ = nh_->advertise<visualization_msgs::Marker>(topic_name.c_str(), 10);

        static visualization_msgs::Marker marker;
        marker.header.frame_id = "map_tmk_opendrive";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Polygon";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 1;
        marker.points.clear();

        for (auto &point :points) {
            static geometry_msgs::Point now_point;
            now_point.x = point(0);
            now_point.y = point(1);
            now_point.z = 0.02;
            marker.points.emplace_back(now_point);
        }
        ros::Rate loop_rate(10);
        for (int i = 1; i <= 10; i++) {
            if (ros::ok()) {
                pub_.publish(marker);
                ros::spinOnce();
                ROS_INFO("ok");
                loop_rate.sleep();
            } else {
                ROS_ERROR("ros not ok");
            }
        }

    }

};  // namespace opendrive_generator 
