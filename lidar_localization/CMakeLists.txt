cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

add_compile_options(-std=c++11)
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
        )
find_package(Eigen3)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

link_directories(lib)
catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/lidar_localization.cpp
        src/lidar_localization_node.cpp
#        include/gps_tools/gpsTools.cpp

        include/lidar_localization/pose_estimator.h
        include/lidar_localization/pose_system.h
        include/lidar_localization/kalman_filter.h
        #        include/utils/pose_estimator.h
        #        include/utils/pose_system.h
        #        include/utils/unscented_kalman_filter.h
        )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenMP_LIBS} ndt_cpu ndt_omp pcl_omp_registration)