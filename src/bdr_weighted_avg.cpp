#include "bdr_weighted_avg.hpp"

std::vector<std::vector<int>> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k){
	std::vector<int> indices;
	std::vector<float> sqDistance;
	std::vector<std::vector<int>> neighborhood;
	
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	
	for (int i = 0; i < cloud->size(); ++i){
		pcl::PointXYZRGB p = (*cloud)[i];
		kdtree.nearestKSearch(p, k, indices, sqDistance);
		neighborhood.push_back(indices);
	}
	
	for (int i = 0; i < neighborhood.size(); ++i){
		for(int index : neighborhood[i]){
			if (std::find(neighborhood[index].begin(), neighborhood[index].end(), i) == neighborhood[index].end())
				neighborhood[index].push_back(i);
		}
	}
	return neighborhood;
}

std::vector<int> RadialNeighbors(pcl::PointXYZRGB search, float radius, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree){
	std::vector<int> indices;
	std::vector<float> sqDistance;
	kdtree.radiusSearch(search, radius, indices, sqDistance);
	return indices;
}

std::vector<int> KnnNeighbors(pcl::PointXYZRGB search, int k, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree){
	std::vector<int> indices;
	std::vector<float> sqDistance;
	kdtree.nearestKSearch(search, k, indices, sqDistance);
	return indices;
}

pcl::PointXYZRGB neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices){
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	for (auto index : indices){
		auto p = (*pc)[index];
		x += p.x;
		y += p.y;
		z += p.z;
	}

	return pcl::PointXYZRGB { x / indices.size(), y / indices.size(), z / indices.size() };
}

float SquaredDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
	return std::sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

void radius_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
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

void knn_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
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

void sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	float tolerance = 0.0015;int k = 40;

	std::vector<int> bdr;
	std::vector<std::vector<int>> neighborhood = SymNeighbors(cloud, k);
	
	for (int i = 0; i < neighborhood.size(); ++i){
		pcl::PointXYZRGB p = (*cloud)[i];
		pcl::PointXYZRGB c = neighbourhood_centroid(cloud, neighborhood[i]);
		if (SquaredDistance(c, p) > tolerance){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
}
