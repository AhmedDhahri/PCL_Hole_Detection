#pragma once 
#include "utils.hpp"
#define PI	3.14159265358979323846

typedef struct{
	public: int index;
	public: float f_redundancy;
	public: float avg_pt_dist;
	std::vector<int> neighbourhood;
}Surface_Info;

std::vector<std::pair<int, float>> get_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<int, std::vector<int>>> set_points_array);

pcl::PointXYZRGB get_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indecies);

float get_pcl_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float resolution, float lambda);

std::vector<Surface_Info> radial_neighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius);

void redundancy_factor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
					std::vector<Surface_Info>& si_array, int reg_inner_pt, float lambda);

void avg_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Surface_Info>& si_array);
