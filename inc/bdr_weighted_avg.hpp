#pragma once 
#include "utils.hpp"

class Weighted_average{

	public:		void radius_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	public:		void knn_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	public:		void sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};
