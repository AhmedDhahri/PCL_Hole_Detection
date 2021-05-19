#include "utils.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_rgb.get()->points.resize(cloud.get()->size());
	for (size_t i = 0; i < cloud_rgb.get()->points.size(); i++) {
		cloud_rgb.get()->points[i].x = cloud.get()->points[i].x;
		cloud_rgb.get()->points[i].y = cloud.get()->points[i].y;
		cloud_rgb.get()->points[i].z = cloud.get()->points[i].z;
		cloud_rgb.get()->points[i].r = 30;
		cloud_rgb.get()->points[i].g = 144;
		cloud_rgb.get()->points[i].b = 255;
	}
	return cloud_rgb;
}

Vector_3D get_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices, pcl::PointXYZRGB c){
	float min_angle = sin(0.09);
	Vector_3D vec_n(c);
	pcl::PointXYZRGB p = cloud.get()->points[indices[0]];
	Vector_3D v(p.x - c.x, p.y - c.y, p.z - c.z);
	for(int i : indices){
		Vector_3D n;
		Vector_3D u = v;
		p = cloud.get()->points[i];
		v = Vector_3D(p.x - c.x, p.y - c.y, p.z - c.z);
		
		n.x = u.y * v.z - u.z * v.y;
		n.y = u.z * v.x - u.x * v.z;
		n.z = u.x * v.y - u.y * v.x;
		if(n.norm()/(u.norm() * v.norm()) > min_angle)
			vec_n = vec_n + n;
	}
	return vec_n/vec_n.norm();
}


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

pcl::PointXYZRGB neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n){
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	for (auto index : indices){
		auto p = (*pc)[index];
		Vector_3D v = Vector_3D(p.x - n.center.x, p.y - n.center.y, p.z - n.center.z);
		float dot_prod = v.x * n.x + v.y * n.y + v.z * n.z;
		
		x += p.x - n.x * dot_prod;//no good concept : always substract the same amount of 3d componment.
		y += p.y - n.y * dot_prod;
		z += p.z - n.z * dot_prod;
	}

	return pcl::PointXYZRGB { x / indices.size(), y / indices.size(), z / indices.size() };
}

float SquaredDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
	return std::sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}
