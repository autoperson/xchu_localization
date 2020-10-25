// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
#include "opendrive_common.h"
#include "param_manager.h"
#include <opendrive_common.h>

namespace opendrive_generator {
    ParamManager::ParamManager() {
        pass_check_ = false;
        Init();
    }

    void ParamManager::Init() {
        pass_check_ = false;
        config_path_ = "/var/params/opendrive_generator/opendrive_generator_param.yaml";
        if (opendrive_common::GetProtoFromASCIIFile<OpendriveGeneratorParam::Param>(
                config_path_, &param_) == false) {
            opendrive_common::Output::Error(
                    "[ParamManagerGenerator::Init]: Can't find the config path of opendrive generator or format error.");
            printf("opendrive_generator_config_path=%s\n", config_path_.c_str());
            ROS_ERROR(
                    "[ParamManagerGenerator::Init]: Can't find the config path of opendrive ge or format error.. Please check opendrive_generator_config_path. opendrive_generator_config_path=%s",
                    config_path_.c_str());
            return;
        }
        if (CheckParam() == false) return;
        OutputParam();
        pass_check_ = true;
    }

    void ParamManager::OutputParam() {
        std::cout << "opendrive_generator_param_list:\n";
        std::cout << "union_roadside_builder.min_segment_length: "
                  << param_.union_roadside_builder().min_segment_length() << std::endl;
        std::cout << "polygon_point_reduction_angle_limit: "
                  << param_.union_roadside_builder().polygon_point_reduction_angle_limit() << std::endl;
        std::cout << std::endl;
    }


    bool ParamManager::CheckParam() const {
        if (param_.union_roadside_builder().min_segment_length() < 0) {
            opendrive_common::Output::Error("generator_param_.union_roadside_builder().min_segment_length()<0");
            return false;
        }
        return true;
    }

    const OpendriveGeneratorParam::Param &ParamManager::GetParam() const {
        return param_;
    }

    bool ParamManager::IfPassCheck() const { return pass_check_; }
};  // namespace opendrive_generator
