// Copyright (c) 2019, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_FILE_WRITER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_FILE_WRITER_H_

#include <string>
#include "hdmap.h"
#include "lane_info_writer/lane_info_writer.h"
#include "roadside_writer/roadside_writer.h"
#include "routemap_writer/routemap_writer.h"
#include "topomap_writer/topomap_writer.h"

namespace opendrive_generator {
    class FileWriter {
    public:
        FileWriter();

        void WriteFiles(const hdmap::EntireMap &entire_map, const std::string &topomap_path,
                        const std::string &routemap_path,
                        const std::string &lane_info_path, const std::string &roadside_path);

    private:
        opendrive_generator::TopomapWriter *topomap_writer_;
        opendrive_generator::RoutemapWriter *routemap_writer_;
        opendrive_generator::LaneInfoWriter *lane_info_writer_;
        opendrive_generator::RoadsideWriter *roadside_writer_;
    };
};      // namespace FileWriterNS
#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_FILE_WRITER_H_
