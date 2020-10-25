// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "lane_info_writer.h"
#include <fstream>

namespace opendrive_generator {
    void LaneInfoWriter::Write(const std::string &file_path, const hdmap::EntireMap &entire_map) {
        if (file_path == "") return;
        lane_info_manager_.Load(entire_map);
        lane_info_manager_.Write(file_path);
    }
};  // namespace opendrive_generator