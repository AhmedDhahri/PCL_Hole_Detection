cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  inc)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(bdr_weighted_avg SHARED src/bdr_weighted_avg.cpp)

add_executable (test_pcl main.cpp)
target_link_libraries (test_pcl ${PCL_LIBRARIES} bdr_weighted_avg)
