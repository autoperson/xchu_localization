// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com
#ifndef SRC_OPENDRIVE_GENERATOR_SRC_PARAM_MANAGER_PARAM_MANAGER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_PARAM_MANAGER_PARAM_MANAGER_H_

#include "opendrive_generator_param.pb.h"
#include "ros/ros.h"

namespace opendrive_generator {
    class ParamManager {
    public:
        ParamManager();

        const OpendriveGeneratorParam::Param &GetParam() const;

        void Init();

        bool IfPassCheck() const;

        static ParamManager &GetInstance(void) {
            static ParamManager this_singleton;
            return this_singleton;
        }

    private:
        std::string config_path_;
        OpendriveGeneratorParam::Param param_;

        void OutputParam();

        bool CheckParam() const;

        bool pass_check_;
    };
};  // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_PARAM_MANAGER_PARAM_MANAGER_H_
