cmake_minimum_required(VERSION 2.8.3)
project(map_loader)

add_compile_options(-std=c++14) # lanelet2需要
find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        geometry_msgs
        pcl_ros
        roscpp
        std_msgs
        tf
        tf2_geometry_msgs
        tf2_ros
        visualization_msgs
        cmake_modules
        pcl_conversions
        )
find_package(PCL REQUIRED COMPONENTS io)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(CURL REQUIRED)
# find_package(TinyXML REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")
catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_IO_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        # ${TinyXML_INCLUDE_DIRS}
)


link_directories(lib) #找到.so


add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} get_file ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
