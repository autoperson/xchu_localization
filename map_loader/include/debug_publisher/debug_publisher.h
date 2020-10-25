// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_DEBUG_PUBLISHER_DEBUG_PUBLISHER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_DEBUG_PUBLISHER_DEBUG_PUBLISHER_H_

#include "opendrive_common.h"
#include "ros/ros.h"
namespace opendrive_generator {
class DebugPublisher
{
 public:
  void InitRos(ros::NodeHandle *nh);
  void PublishPolygon(const std::string &topic_name,
                      const opendrive_common::GeometryPolygon &polygon);
  void PublishPoints(const std::string &topic_name,
                      const std::vector<opendrive_common::GeometryPoint> &points);
  void PublishEdges(const std::string &topic_name,
                      const std::vector<opendrive_common::GeometryPoint> &points);
  static DebugPublisher &GetInstance(void) {
    static DebugPublisher this_singleton;
    return this_singleton;
  }
 private:
  
  ros::NodeHandle *nh_;
};
};      // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_DEBUG_PUBLISHER_DEBUG_PUBLISHER_H_
