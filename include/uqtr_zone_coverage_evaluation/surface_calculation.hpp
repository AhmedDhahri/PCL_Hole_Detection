#pragma once 
#include "uqtr_zone_coverage_evaluation/utils.hpp"
#define PI	3.14159265358979323846

typedef struct{
	public: int index;
	public: float f_redundancy;
	public: float avg_pt_dist;
	std::vector<int> neighbourhood;
}Surface_Info;

void rm_out_border(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
					std::vector<std::pair<int, std::vector<int>>>& set_points_array,
					std::vector<Point_info> pi_array,
					int k, int min_bdr);
					
std::vector<std::pair<int, float>> get_hole_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<int, std::vector<int>>> set_points_array);

pcl::PointXYZRGB get_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indecies);

float get_pcl_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float resolution);

std::vector<Surface_Info> radial_neighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius);

void redundancy_factor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
					std::vector<Surface_Info>& si_array, int reg_inner_pt, float radius);

void avg_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Surface_Info>& si_array);
