cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  inc)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_library(pcl_utils SHARED src/utils.cpp)
add_library(bdr_angle SHARED src/bdr_angle.cpp)
add_library(bdr_weighted_avg SHARED src/bdr_weighted_avg.cpp)
add_library(kruskal_mst SHARED src/kruskal_mst.cpp)
add_library(surface_calculation SHARED src/surface_calculation.cpp)

#add_dependencies(bdr_weighted_avg utils)
target_link_libraries(bdr_angle pcl_utils)
target_link_libraries(bdr_weighted_avg pcl_utils)
target_link_libraries(kruskal_mst pcl_utils)
target_link_libraries(surface_calculation pcl_utils)

add_executable (test_pcl main.cpp)
target_link_libraries(test_pcl ${PCL_LIBRARIES} bdr_weighted_avg bdr_angle pcl_utils kruskal_mst surface_calculation)
