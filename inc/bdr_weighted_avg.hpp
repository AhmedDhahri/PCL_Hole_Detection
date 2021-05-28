#pragma once 
#include "utils.hpp"


void radius_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void knn_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k);
std::vector<int> sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::vector<int>> neighborhood);

