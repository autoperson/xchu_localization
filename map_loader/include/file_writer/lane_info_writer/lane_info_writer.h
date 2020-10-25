// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_LANE_INFO_WRITER_LANE_INFO_WRITER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_LANE_INFO_WRITER_LANE_INFO_WRITER_H_

#include <string>
#include "hdmap.h"

namespace opendrive_generator {
    class LaneInfoWriter {
    public:
        void Write(const std::string &file_path, const hdmap::EntireMap &entire_map);

        static LaneInfoWriter &GetInstance(void) {
            static LaneInfoWriter this_singleton;
            return this_singleton;
        }

    private:
        hdmap::LaneInfoManager lane_info_manager_;
    };
};  // namespace LanesInfoNS

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_LANE_INFO_WRITER_LANE_INFO_WRITER_H_
