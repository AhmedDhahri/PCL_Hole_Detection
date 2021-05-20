#include "bdr_weighted_avg.hpp"


void Weighted_average::radius_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	float radius = 0.015f, tolerance = 0.006;
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> bdr;
	std::cout<<cloud->size()<<std::endl;
	for (int i = 0; i < cloud->size(); ++i){
		pcl::PointXYZRGB p = (*cloud)[i];
		auto res = RadialNeighbors(p, radius, kdtree);
		pcl::PointXYZRGB c = neighbourhood_centroid(cloud, res);
		if (SquaredDistance(c, p) > tolerance){
			//bdr.push_back(i);
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
}

void Weighted_average::knn_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	float tolerance = 0.0015;int k = 40;
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> bdr;
	std::cout<<cloud->size()<<std::endl;
	for (int i = 0; i < cloud->size(); ++i){
		pcl::PointXYZRGB p = (*cloud)[i];
		auto res = KnnNeighbors(p, k, kdtree);
		pcl::PointXYZRGB c = neighbourhood_centroid(cloud, res);
		if (SquaredDistance(c, p) > tolerance){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
}

void Weighted_average::sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	float tolerance = 0.0010;int k = 40;

	std::vector<int> bdr;
	std::vector<std::vector<int>> neighborhood = SymNeighbors(cloud, k);
	
	for (int i = 0; i < neighborhood.size(); ++i){
		Vector_3D n = get_normal_pca(cloud, neighborhood[i], cloud.get()->points[i]);
		n = get_normal(cloud, neighborhood[i], cloud.get()->points[i]);///////////////////////
		
		pcl::PointXYZRGB p = (*cloud)[i];
		pcl::PointXYZRGB c = neighbourhood_centroid_proj(cloud, neighborhood[i], n);
		if (SquaredDistance(c, p) > tolerance){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
}

