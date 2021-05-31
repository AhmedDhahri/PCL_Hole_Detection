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

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices){
	
	Vector_3D vec_n;
	double mean_x = 0, mean_y = 0, mean_z = 0;
	for(int k=0; k<indices.size();k++) {
		mean_x += (cloud.get()->points[indices[k]].x - cloud.get()->points[indices[0]].x);
		mean_y += (cloud.get()->points[indices[k]].y - cloud.get()->points[indices[0]].y);
		mean_z += (cloud.get()->points[indices[k]].z - cloud.get()->points[indices[0]].z);
	}
	mean_x /= indices.size();
	mean_y /= indices.size();
	mean_z /= indices.size();
	
	Eigen::Matrix3d covar_matrix;
	covar_matrix(0,0) = 0;
	covar_matrix(0,1) = 0;
	covar_matrix(0,2) = 0;
	covar_matrix(1,1) = 0;
	covar_matrix(1,2) = 0;
	covar_matrix(2,2) = 0;
	
	for(int k=0; k<indices.size();k++) {
		covar_matrix(0,0) += ((cloud.get()->points[indices[k]].x - cloud.get()->points[indices[0]].x) - mean_x) * ((cloud.get()->points[indices[k]].x - cloud.get()->points[indices[0]].x) - mean_x);
		covar_matrix(0,1) += ((cloud.get()->points[indices[k]].y - cloud.get()->points[indices[0]].y) - mean_y) * ((cloud.get()->points[indices[k]].x - cloud.get()->points[indices[0]].x) - mean_x);
		covar_matrix(0,2) += ((cloud.get()->points[indices[k]].z - cloud.get()->points[indices[0]].z) - mean_z) * ((cloud.get()->points[indices[k]].x - cloud.get()->points[indices[0]].x) - mean_x);
		covar_matrix(1,1) += ((cloud.get()->points[indices[k]].y - cloud.get()->points[indices[0]].y) - mean_y) * ((cloud.get()->points[indices[k]].y - cloud.get()->points[indices[0]].y) - mean_y);
		covar_matrix(1,2) += ((cloud.get()->points[indices[k]].z - cloud.get()->points[indices[0]].z) - mean_z) * ((cloud.get()->points[indices[k]].y - cloud.get()->points[indices[0]].y) - mean_y);
		covar_matrix(2,2) += ((cloud.get()->points[indices[k]].z - cloud.get()->points[indices[0]].z) - mean_z) * ((cloud.get()->points[indices[k]].z - cloud.get()->points[indices[0]].z) - mean_z);
	}
	covar_matrix(0,0) /= indices.size();
	covar_matrix(0,1) /= indices.size();
	covar_matrix(0,2) /= indices.size();
	covar_matrix(1,1) /= indices.size();
	covar_matrix(1,2) /= indices.size();
	covar_matrix(2,2) /= indices.size();
	
	covar_matrix(1,0) = covar_matrix(0,1);
	covar_matrix(2,0) = covar_matrix(0,2);
	covar_matrix(2,1) = covar_matrix(1,2);
	
	Eigen::EigenSolver<Eigen::Matrix3d> es;
	es.compute(covar_matrix);
	Eigen::Vector3cd eval = es.eigenvalues();
	Eigen::Matrix3cd evec = es.eigenvectors();
	

	if((eval(0).real() <= eval(1).real()) && (eval(0).real() <= eval(2).real())){
		vec_n.x = evec(0,0).real();
		vec_n.y = evec(1,0).real();
		vec_n.z = evec(2,0).real();
	}
	else if((eval(1).real() <= eval(0).real()) && (eval(1).real() <= eval(2).real())){
		vec_n.x = evec(0,1).real();
		vec_n.y = evec(1,1).real();
		vec_n.z = evec(2,1).real();
	}
	else if((eval(2).real() <= eval(1).real()) && (eval(2).real() <= eval(0).real())){
		vec_n.x = evec(0,2).real();
		vec_n.y = evec(1,2).real();
		vec_n.z = evec(2,2).real();
	}
	return vec_n/vec_n.norm();
}

std::vector<Point_info> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k){
	std::vector<int> indices;
	std::vector<float> sqDistance;
	std::vector<Point_info> pcl_info;
	
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	
	for (int i = 0; i < cloud->size(); ++i){
		Point_info pi;
		pi.index = i;
		pcl::PointXYZRGB p = (*cloud)[i];
		kdtree.nearestKSearch(p, k, pi.neighbourhood, sqDistance);
		pcl_info.push_back(pi);
	}
	
	for (int i = 0; i < pcl_info.size(); ++i){////
		for(int u : pcl_info[i].neighbourhood){
			if (std::find(pcl_info[u].neighbourhood.begin(), pcl_info[u].neighbourhood.end(), i) 
																	== pcl_info[u].neighbourhood.end())
				pcl_info[u].neighbourhood.push_back(i);
		}
	}
	return pcl_info;
}

Centroid_Info neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices){
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	float xt, yt, zt;
	float density = 0;
	auto c = (*pc)[indices[0]];
	
	for (auto index : indices){
		auto p = (*pc)[index];
		xt = p.x - c.x;
		yt = p.y - c.y;
		zt = p.z - c.z;
		
		x += xt;
		y += yt;
		z += zt;
		density += std::sqrt(xt*xt + yt*yt + zt*zt);
	}

	return {density/indices.size(), pcl::PointXYZRGB { x/indices.size(), y/indices.size(), z/indices.size()}};
}

Centroid_Info neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n){
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	float xt, yt, zt;
	float density = 0;
	for (auto index : indices){
		auto p = (*pc)[index];
		Vector_3D v = Vector_3D(p.x - (*pc)[indices[0]].x, p.y - (*pc)[indices[0]].y, p.z - (*pc)[indices[0]].z);
		float dot_prod = v.x * n.x + v.y * n.y + v.z * n.z;
		
		xt = v.x - n.x * dot_prod;
		yt = v.y - n.y * dot_prod;
		zt = v.z - n.z * dot_prod;
		
		x += xt;y += yt; z+= zt;
		
		density += std::sqrt(xt*xt + yt*yt + zt*zt);
	}

	return {density/indices.size(), pcl::PointXYZRGB { x/indices.size(), y/indices.size(), z/indices.size()}};
}

void make_graph(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, std::vector<int> bdr_wavg, std::vector<int> bdr_angl){
	
}
