#pragma once 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
