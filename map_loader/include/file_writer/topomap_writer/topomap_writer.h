// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_TOPOMAP_WRITER_TOPOMAP_WRITER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_TOPOMAP_WRITER_TOPOMAP_WRITER_H_

#include <string>
#include "hdmap.h"
#include <vector>

namespace opendrive_generator {
    class TopomapWriter {
    public:
        void Write(const std::string &file_path, const hdmap::EntireMap &map_manager);

        static TopomapWriter &GetInstance(void) {
            static TopomapWriter this_singleton;
            return this_singleton;
        }

        const std::vector <hdmap::TopoNodePtr> &node_list() const;

    private:
        hdmap::Topomap topomap_;

    };  // namespace topomap
};  // namespace opendrive_generator
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_TOPOMAP_WRITER_TOPOMAP_WRITER_H_
