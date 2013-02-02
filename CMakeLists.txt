cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write src/pcd_write.cpp)
add_executable (kitti_pcl src/kitti_pcl.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
target_link_libraries (kitti_pcl ${PCL_LIBRARIES})
