cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_test)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_test src/pcl_test.cpp)
#add_executable (kitti_pcl src/kitti_pcl.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})
#target_link_libraries (kitti_pcl ${PCL_LIBRARIES})
