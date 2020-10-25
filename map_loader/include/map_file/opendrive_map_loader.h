//
// Created by xchu on 2020/5/19.
//

#ifndef SRC_OPENDRIVE_MAP_LOADER_H
#define SRC_OPENDRIVE_MAP_LOADER_H

#include <ros/ros.h>
//#include <vector_map/vector_map.h>
//#include "libvectormap/vector_map.h"
//#include "libvectormap/Math.h"

//#include <autoware_msgs/AdjustXY.h>
//#include <autoware_msgs/Lane.h>
//#include <autoware_msgs/Signals.h>
//#include <tf/tf.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseArray.h"

//#include "vector_map_msgs/PointArray.h"
//#include "vector_map_msgs/LaneArray.h"
//#include "vector_map_msgs/NodeArray.h"
//#include "vector_map_msgs/StopLineArray.h"
//#include "vector_map_msgs/DTLaneArray.h"
//#include "vector_map_msgs/LineArray.h"
//#include "vector_map_msgs/AreaArray.h"
//#include "vector_map_msgs/SignalArray.h"
//#include "vector_map_msgs/StopLine.h"
//#include "vector_map_msgs/VectorArray.h"
//#include "vector_map_msgs/RoadEdgeArray.h"
//
#include "op_utility/DataRW.h"
#include "file_writer/file_writer.h"

//#include "op_planner/PlannerCommonDef.h"
//#include "op_planner/MappingHelpers.h"
//#include "op_planner/PlannerH.h"
//#include "op_ros_helpers/op_ROSHelpers.h" // 在common

#include "hdmap.h" // opendrive支持
#include "opendrive_common.h"

#include "file_writer/roadside_writer/roadside_writer.h"
#include "file_writer/topomap_writer/topomap_writer.h"

namespace opendrive_map_loader {

    enum Color : int {
        BLACK,
        GRAY,
        LIGHT_RED,
        LIGHT_GREEN,
        LIGHT_BLUE,
        LIGHT_YELLOW,
        LIGHT_CYAN,
        LIGHT_MAGENTA,
        RED,
        GREEN,
        BLUE,
        YELLOW,
        CYAN,
        MAGENTA,
        WHITE
    };

    class WayPlannerParams {
    public:
        std::string KmlMapPath;
//        bool bEnableSmoothing;
//        bool bEnableLaneChange;
//        bool bEnableHMI;
//        bool bEnableRvizInput;
//        bool bEnableReplanning;
//        double pathDensity;
//        PlannerHNS::MAP_SOURCE_TYPE mapSource;
//        bool bEnableDynamicMapUpdate;


        WayPlannerParams() {
//            bEnableDynamicMapUpdate = false;
//            bEnableReplanning = false;
//            bEnableHMI = false;
//            bEnableSmoothing = false;
//            bEnableLaneChange = false;
//            bEnableRvizInput = true;
//            pathDensity = 0.5;
//            mapSource = PlannerHNS::MAP_KML_FILE;
        }
    };

    class OpenDriveMapLoader {
    public:
        OpenDriveMapLoader(ros::NodeHandle &node_handle);

        void MainLoop();

//        VectorMap vmap;
        WayPlannerParams m_params;

        std_msgs::ColorRGBA createColorRGBA(Color color);
        /*inline double ConvertDegreeToRadian(double degree) {
            return degree * M_PI / 180.0f;
        }

        inline double ConvertRadianToDegree(double radian) {
            return radian * 180.0f / M_PI;
        }*/

    private:

        // opendrive viewer
        void PubReferenceLineForVistualize();

        void PubEdgeForVistualize();

        void PubLineNodeForVistualize();

        void PubLaneNodeForVistualize();

        //tmk_tmp void PubEdgeForVistualize();
        void PubRoadInfoForVistualize();

        void PubLaneConnectionForVistualize();

        void PubRoadSideForVistualize();

        void PubSignForVistualize();

        void PubTopomapForVistualize();

        // node
        ros::NodeHandle &nodeHandle_;

        // opendrive
        ros::Publisher reference_line_pub_;
        ros::Publisher line_node_pub_;
        ros::Publisher lane_node_pub_;
        ros::Publisher edge_pub_;
        ros::Publisher road_info_pub_;
        ros::Publisher lane_connection_pub_;
        ros::Publisher roadside_pub_;
        ros::Publisher sign_pub_;
        ros::Publisher topomap_pub_;

        // opendrive support
        hdmap::EntireMap entire_map_;
        opendrive_generator::TopomapWriter topomap_writer_;
        opendrive_generator::RoadsideWriter roadside_writer_;

        opendrive_generator::FileWriter file_writer_;


        std::string map_path;
    };

} // namespace signal_service


#endif //SRC_OPENDRIVE_MAP_LOADER_H
