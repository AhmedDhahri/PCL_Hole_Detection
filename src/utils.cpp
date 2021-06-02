#include "utils.hpp"

Vector_3D::Vector_3D():x(0),y(0),z(0){
}

Vector_3D::Vector_3D(float xt, float yt, float zt){
	x=xt;
	y=yt;
	z=zt;
}
	
float Vector_3D::norm(){
	return std::sqrt(x*x + y*y + z*z);
}

Vector_3D Vector_3D::operator /(float cst){
	x /= cst;
	y /= cst;
	z /= cst;
	Vector_3D* n = this;
	return *n;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_rgb->resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); i++) {
		(*cloud_rgb)[i].x = (*cloud)[i].x;
		(*cloud_rgb)[i].y = (*cloud)[i].y;
		(*cloud_rgb)[i].z = (*cloud)[i].z;

		
		(*cloud_rgb)[i].r = 30;
		(*cloud_rgb)[i].g = 144;
		(*cloud_rgb)[i].b = 255;
	}
	return cloud_rgb;
}

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices){
	
	Vector_3D vec_n;
	double mean_x = 0, mean_y = 0, mean_z = 0;
	for(int k=0; k<indices.size();k++) {
		mean_x += ((*cloud)[indices[k]].x - (*cloud)[indices[0]].x);
		mean_y += ((*cloud)[indices[k]].y - (*cloud)[indices[0]].y);
		mean_z += ((*cloud)[indices[k]].z - (*cloud)[indices[0]].z);
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
		covar_matrix(0,0) += (((*cloud)[indices[k]].x - (*cloud)[indices[0]].x) - mean_x) * (((*cloud)[indices[k]].x - (*cloud)[indices[0]].x) - mean_x);
		covar_matrix(0,1) += (((*cloud)[indices[k]].y - (*cloud)[indices[0]].y) - mean_y) * (((*cloud)[indices[k]].x - (*cloud)[indices[0]].x) - mean_x);
		covar_matrix(0,2) += (((*cloud)[indices[k]].z - (*cloud)[indices[0]].z) - mean_z) * (((*cloud)[indices[k]].x - (*cloud)[indices[0]].x) - mean_x);
		covar_matrix(1,1) += (((*cloud)[indices[k]].y - (*cloud)[indices[0]].y) - mean_y) * (((*cloud)[indices[k]].y - (*cloud)[indices[0]].y) - mean_y);
		covar_matrix(1,2) += (((*cloud)[indices[k]].z - (*cloud)[indices[0]].z) - mean_z) * (((*cloud)[indices[k]].y - (*cloud)[indices[0]].y) - mean_y);
		covar_matrix(2,2) += (((*cloud)[indices[k]].z - (*cloud)[indices[0]].z) - mean_z) * (((*cloud)[indices[k]].z - (*cloud)[indices[0]].z) - mean_z);
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
	std::cout<<"Computing neighbourhood..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	
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
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Computing neighbourhood finished clearly in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	return pcl_info;
}

Centroid_Info neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices){
	Vector_3D c;
	float density = 0;
	
	for (auto index : indices){
		c.x += ((*pc)[index].x - (*pc)[indices[0]].x);
		c.y += ((*pc)[index].y - (*pc)[indices[0]].y);
		c.z += ((*pc)[index].z - (*pc)[indices[0]].z);
		density += std::sqrt(	((*pc)[index].x - (*pc)[indices[0]].x)*((*pc)[index].x - (*pc)[indices[0]].x) + 
								((*pc)[index].y - (*pc)[indices[0]].y)*((*pc)[index].y - (*pc)[indices[0]].y) + 
								((*pc)[index].z - (*pc)[indices[0]].z)*((*pc)[index].z - (*pc)[indices[0]].z));
	}

	return {density/indices.size(), c/indices.size() };
}

Centroid_Info neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n){
	Vector_3D c;
	float xt, yt, zt;
	float density = 0;
	for (auto index : indices){
		auto p = (*pc)[index];
		Vector_3D v = Vector_3D(p.x - (*pc)[indices[0]].x, p.y - (*pc)[indices[0]].y, p.z - (*pc)[indices[0]].z);
		float dot_prod = v.x * n.x + v.y * n.y + v.z * n.z;
		
		xt = v.x - n.x * dot_prod;
		yt = v.y - n.y * dot_prod;
		zt = v.z - n.z * dot_prod;
		
		c.x += xt;c.y += yt; c.z+= zt;
		
		density += std::sqrt(xt*xt + yt*yt + zt*zt);
	}

	return {density/indices.size(), c/indices.size() };
}

Array_saver::Array_saver(std::string path){
	this->path = path;
}

void Array_saver::dump(std::vector<Point_info> pi_array){
	std::cout<<"Point array dumping..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	std::ofstream file (path, ios::binary|ios::out);
	//pi_array_size
	long s = pi_array.size();
	char* buffer = new char[sizeof(long)];
	memcpy(buffer, &s, sizeof(long));
	file.write (buffer, sizeof(long));	
	
	for(Point_info p : pi_array){
		//index
		buffer = new char[sizeof(int)];
		memcpy(buffer, &(p.index), sizeof(int));
		file.write (buffer, sizeof(int));	
		//density
		buffer = new char[sizeof(float)];
		memcpy(buffer, &(p.density), sizeof(float));
		file.write (buffer, sizeof(float));	
		//wtavg_prob
		buffer = new char[sizeof(float)];
		memcpy(buffer, &(p.wtavg_prob), sizeof(float));
		file.write (buffer, sizeof(float));	
		//angle_prob
		buffer = new char[sizeof(float)];
		memcpy(buffer, &(p.angle_prob), sizeof(float));
		file.write (buffer, sizeof(float));	
		//bdr
		buffer = new char[sizeof(bool)];
		memcpy(buffer, &(p.bdr), sizeof(bool));
		file.write (buffer, sizeof(bool));	
		//wa
		buffer = new char[sizeof(bool)];
		memcpy(buffer, &(p.wa), sizeof(bool));
		file.write (buffer, sizeof(bool));	
		//ag
		buffer = new char[sizeof(bool)];
		memcpy(buffer, &(p.ag), sizeof(bool));
		file.write (buffer, sizeof(bool));	
		//included_pts
		buffer = new char[sizeof(Vector_3D)];
		memcpy(buffer, &(p.normal), sizeof(Vector_3D));
		file.write (buffer, sizeof(Vector_3D));
		
		//neighbourhood_size
		s = p.neighbourhood.size();
		buffer = new char[sizeof(long)];
		memcpy(buffer, &s, sizeof(long));
		file.write (buffer, sizeof(long));
		//neighbourhood
		for(int i : p.neighbourhood){
			buffer = new char[sizeof(int)];
			memcpy(buffer, &i, sizeof(int));
			file.write (buffer, sizeof(int));
		}
		//included_pts_size
		s = p.included_pts.size();
		buffer = new char[sizeof(long)];
		memcpy(buffer, &s, sizeof(long));
		file.write (buffer, sizeof(long));
		//included_pts
		for(int i : p.included_pts){
			buffer = new char[sizeof(int)];
			memcpy(buffer, &i, sizeof(int));
			file.write (buffer, sizeof(int));
		}
	}
	delete buffer;
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Point array dumping finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
}

void Array_saver::load(std::vector<Point_info>& pi_array){
	std::cout<<"Point array loading..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	std::ifstream file (path, ios::in|ios::binary);
	if (file.is_open()){
		std::cout << "File opened successfully."<<std::endl;
		long s;char* buffer;
		
		//pi_array_size
		long pi_array_size;
		buffer = new char[sizeof(long)];
		file.read (buffer, sizeof(long));	
		memcpy(&pi_array_size, buffer, sizeof(long));
		
		for(long i=0;i<pi_array_size;i++){
			Point_info p;
			//index
			buffer = new char[sizeof(int)];
			file.read (buffer, sizeof(int));	
			memcpy(&(p.index), buffer, sizeof(int));
			
		
			//density
			buffer = new char[sizeof(float)];
			file.read (buffer, sizeof(float));	
			memcpy(&(p.density), buffer, sizeof(float));
			
		
			//wtavg_prob
			buffer = new char[sizeof(float)];
			file.read (buffer, sizeof(float));	
			memcpy(&(p.wtavg_prob), buffer, sizeof(float));
			
		
			//angle_prob
			buffer = new char[sizeof(float)];
			file.read (buffer, sizeof(float));		
			memcpy(&(p.angle_prob), buffer, sizeof(float));
			
		
			//bdr
			buffer = new char[sizeof(bool)];
			file.read (buffer, sizeof(bool));	
			memcpy(&(p.bdr), buffer, sizeof(bool));
			
		
			//wa
			buffer = new char[sizeof(bool)];
			file.read (buffer, sizeof(bool));	
			memcpy(&(p.wa), buffer, sizeof(bool));
			
		
			//ag
			buffer = new char[sizeof(bool)];
			file.read (buffer, sizeof(bool));	
			memcpy(&(p.ag), buffer, sizeof(bool));
			
		
			//normal
			buffer = new char[sizeof(Vector_3D)];
			file.read (buffer, sizeof(Vector_3D));	
			memcpy(&(p.normal), buffer, sizeof(Vector_3D));
			
		
			//neighbourhood_size
			buffer = new char[sizeof(long)];
			file.read (buffer, sizeof(long));	
			memcpy(&s, buffer, sizeof(long));
			
		
			//neighbourhood
			for(long j=0;j<s;j++){
				int k;
				buffer = new char[sizeof(int)];
				file.read (buffer, sizeof(int));	
				memcpy(&k, buffer, sizeof(int));
				p.neighbourhood.push_back(k);
				
			}
			//included_pts_size
			buffer = new char[sizeof(long)];
			file.read (buffer, sizeof(long));	
			memcpy(&s, buffer, sizeof(long));
			
		
			//included_pts
			for(long j=0;j<s;j++){
				int k;
				buffer = new char[sizeof(int)];
				file.read (buffer, sizeof(int));	
				memcpy(&k, buffer, sizeof(int));
				p.included_pts.push_back(k);
				
			}
			pi_array.push_back(p);
		}
		delete buffer;
		float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
		std::cout<<"Point array loading finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	}
	else{
		std::cout << "Faild to open file."<<std::endl;
	}
}
