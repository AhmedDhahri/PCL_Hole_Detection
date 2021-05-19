#include "bdr_angle.hpp"

void sym_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	int k = 40;

	std::vector<int> bdr;
	std::vector<std::vector<int>> neighborhood = SymNeighbors(cloud, k);
	
	for (int i = 0; i < neighborhood.size(); ++i){
		
		pcl::PointXYZRGB p = (*cloud)[i];
		pcl::PointXYZRGB c = neighbourhood_centroid(cloud, neighborhood[i]);
		if (SquaredDistance(c, p) > 0){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
	
}
void sym_angle_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	int k = 40;

	std::vector<int> bdr;
	std::vector<std::vector<int>> neighborhood = SymNeighbors(cloud, k);
	
	for (int i = 0; i < neighborhood.size(); ++i){
		Vector_3D n = get_normal(cloud, neighborhood[i], cloud.get()->points[i]);
		
		pcl::PointXYZRGB p = (*cloud)[i];
		pcl::PointXYZRGB c = neighbourhood_centroid(cloud, neighborhood[i]);
		if (SquaredDistance(c, p) > 0){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
	
}
