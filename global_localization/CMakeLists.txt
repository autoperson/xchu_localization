cmake_minimum_required(VERSION 3.0.2)
project(global_localization)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)
find_package(OpenMP REQUIRED) #链接ndt_cpu库 get_max_thread_num
if (OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
    set(OpenMP_LIBS gomp)
endif ()

find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        pcl_conversions
        pcl_ros
        roscpp
        std_msgs
        tf
        tf_conversions
        eigen_conversions
        image_transport
        )
find_package(Eigen3)
#find_package(teaserpp REQUIRED)
#find_package(OpenCV REQUIRED QUIET)
find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    list(APPEND ThirdParty_LIBS ${OpenCV_LIBS})
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OpenCV_FOUND)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        #        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(
        lib
        ${PCL_LIBRARY_DIRS}
        #        ${OpenCV_LIBRARY_DIRS}
)

add_executable(points_prefilter_node src/points_prefilter.cpp src/node/points_prefilter_node.cpp)
target_link_libraries(points_prefilter_node ${catkin_LIBRARIES})

#add_executable(scan_context_node src/node/scan_context_node.cpp)
#target_link_libraries(scan_context_node ${catkin_LIBRARIES})

#add_executable(test_node src/node/test_sc_node.cpp include/scan_context/Scancontext.cpp)
#target_link_libraries(test_node ${catkin_LIBRARIES})

#add_executable(sc_localization_node src/node/sc_localization_node.cpp src/sc_localization.cpp
#        include/scan_context/Scancontext.cpp
#        include/gps_tools/gpsTools.cpp
#        )
#target_link_libraries(sc_localization_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} #[[${OpenCV_LIBRARIES}]] ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node
        src/global_localization.cpp
        src/node/global_localization_node.cpp
        include/utils/registrations.cpp
        include/utils/transform.h
        include/gps_tools/gpsTools.cpp
        include/global_localization/cloud_data.hpp
        include/global_localization/gnss_data.hpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenMP_LIBS} ndt_cpu ndt_omp pcl_omp_registration)

#add_executable(gps_init
#        src/node/gps_init_node.cpp
#        src/gps_init.cc)
#target_link_libraries(gps_init ${catkin_LIBRARIES})