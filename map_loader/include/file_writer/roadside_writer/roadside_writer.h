// Copyright (c) 2020, Mingkai Tang. All rights reserved.
// Author: Mingkai Tang 910874332@qq.com

#ifndef SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_ROADSIDE_WRITER_H_
#define SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_ROADSIDE_WRITER_H_

#include <string>
#include "hdmap.h"
#include "factory_roadside_builder/factory_roadside_builder.h"

namespace opendrive_generator {

    class RoadsideWriter {
    public:
        RoadsideWriter();

        void Write(const std::string &file_path, const hdmap::EntireMap &entire_map);

        static RoadsideWriter &GetInstance(void) {
            static RoadsideWriter this_singleton;
            return this_singleton;
        }

        std::vector <opendrive_common::GeometryPolygon> polygons_;
    private:
        void WriteFile(const std::string &file_path, const std::vector <opendrive_common::GeometryPolygon> &polygons);

        roadside::FactoryRoadsideBuilder factory_roadside_builder_;
    };
};  // namespace opendrive_generator

#endif  // SRC_OPENDRIVE_GENERATOR_SRC_FILE_WRITER_ROADSIDE_WRITER_ROADSIDE_WRITER_H_
