#pragma once 
#include "utils.hpp"
#define PI	3.14159265358979323846



void sym_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info> pi_array, float tolerance);

double max_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point_info pi);
