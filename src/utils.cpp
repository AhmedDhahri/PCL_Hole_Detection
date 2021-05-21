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

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices, pcl::PointXYZRGB c){
	
	const int dim = indices.size();
	Vector_3D vec_n(c);
	Eigen::MatrixXd data_mat(dim,3);
	double mean_x = 0, mean_y = 0, mean_z = 0;
	for(int k=0; k<dim;k++) {
		data_mat(k,0) = cloud.get()->points[indices[k]].x - c.x;
		data_mat(k,1) = cloud.get()->points[indices[k]].y - c.y;
		data_mat(k,2) = cloud.get()->points[indices[k]].z - c.z;
		mean_x += data_mat(k,0);
		mean_y += data_mat(k,1);
		mean_z += data_mat(k,2);
	}
	mean_x /= dim;
	mean_y /= dim;
	mean_z /= dim;
	
	Eigen::Matrix3d covar_matrix;
	double cov_xx = 0, cov_xy = 0, cov_xz = 0, cov_yy = 0, cov_yz = 0, cov_zz = 0;
	for(int k=0; k<indices.size();k++) {
		cov_xx += (data_mat(k,0) - mean_x) * (data_mat(k,0) - mean_x);
		cov_xy += (data_mat(k,1) - mean_y) * (data_mat(k,0) - mean_x);
		cov_xz += (data_mat(k,2) - mean_z) * (data_mat(k,0) - mean_x);
		cov_yy += (data_mat(k,1) - mean_y) * (data_mat(k,1) - mean_y);
		cov_yz += (data_mat(k,2) - mean_z) * (data_mat(k,1) - mean_y);
		cov_zz += (data_mat(k,2) - mean_z) * (data_mat(k,2) - mean_z);
	}
	cov_xx /= dim;
	cov_xy /= dim;
	cov_xz /= dim;
	cov_yy /= dim;
	cov_yz /= dim;
	cov_zz /= dim;
	covar_matrix(0,0) = cov_xx;
	covar_matrix(0,1) = cov_xy;
	covar_matrix(0,2) = cov_xz;
	covar_matrix(1,0) = cov_xy;
	covar_matrix(1,1) = cov_yy;
	covar_matrix(1,2) = cov_yz;
	covar_matrix(2,0) = cov_xz;
	covar_matrix(2,1) = cov_yz;
	covar_matrix(2,2) = cov_zz;
	
	Eigen::EigenSolver<Eigen::Matrix3d> es;
	es.compute(covar_matrix);
	Eigen::Vector3cd eval = es.eigenvalues();
	Eigen::Matrix3cd evec = es.eigenvectors();
	
	double lambda_1 = eval(0).real();
	double lambda_2 = eval(1).real();
	double lambda_3 = eval(2).real();
	if((lambda_1 <= lambda_2) && (lambda_1 <= lambda_3)){
		vec_n.x = evec(0,0).real();
		vec_n.y = evec(1,0).real();
		vec_n.z = evec(2,0).real();
	}
	else if((lambda_2 <= lambda_1) && (lambda_2 <= lambda_3)){
		vec_n.x = evec(0,1).real();
		vec_n.y = evec(1,1).real();
		vec_n.z = evec(2,1).real();
	}
	else if((lambda_3 <= lambda_2) && (lambda_3 <= lambda_1)){
		vec_n.x = evec(0,2).real();
		vec_n.y = evec(1,2).real();
		vec_n.z = evec(2,2).real();
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
