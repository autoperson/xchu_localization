// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#include "topomap_writer.h"

#include <fstream>

namespace opendrive_generator {
    const std::vector <hdmap::TopoNodePtr> &TopomapWriter::node_list() const { return topomap_.node_list(); }

    void TopomapWriter::Write(const std::string &file_path,
                              const hdmap::EntireMap &entire_map) {
        if (file_path == "") return;
        topomap_.Load(entire_map);
        topomap_.Write(file_path);
    }
};  // namespace opendrive_generator