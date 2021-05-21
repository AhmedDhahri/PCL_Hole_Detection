#include "bdr_angle.hpp"

void sym_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	int k = 40;

	std::vector<int> bdr;
	std::vector<std::vector<int>> neighborhood = SymNeighbors(cloud, k);
	
	for (int i = 0; i < neighborhood.size(); ++i){
		Vector_3D n = get_normal(cloud, neighborhood[i], cloud.get()->points[i]);
		
		pcl::PointXYZRGB c = cloud.get()->points[i];
		float angle = max_angle(cloud, neighborhood[i], n);
		if (angle > 2){
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
	}
}

double max_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices, Vector_3D n){
	Vector_3D u((cloud.get()->points[indices[1]].x - cloud.get()->points[indices[0]].x),
				(cloud.get()->points[indices[1]].y - cloud.get()->points[indices[0]].y),
				(cloud.get()->points[indices[1]].z - cloud.get()->points[indices[0]].z));//axix
	float dot_prod_u = u.x * n.x + u.y * n.y + u.z * n.z;
	u.x = u.x - n.x * dot_prod_u;
	u.y = u.y - n.y * dot_prod_u;
	u.z = u.z - n.z * dot_prod_u;
	u = u / u.norm();
	
	Vector_3D v(n.y*u.z-n.z*u.y,
				n.z*u.x-n.x*u.z,
				n.x*u.y-n.y*u.x);//ord
				
	std::vector<double> angles;
	for (int i = 2; i < indices.size(); ++i){
		Vector_3D a((cloud.get()->points[indices[i]].x - cloud.get()->points[indices[0]].x),
					(cloud.get()->points[indices[i]].y - cloud.get()->points[indices[0]].y),
					(cloud.get()->points[indices[i]].z - cloud.get()->points[indices[0]].z));
					
		float dot_prod_u = a.x * u.x + a.y * u.y + a.z * u.z;
		float dot_prod_v = a.x * v.x + a.y * v.y + a.z * v.z;
		
		
		double angle = acos(dot_prod_u/std::sqrt(std::pow(dot_prod_u, 2) + std::pow(dot_prod_v, 2)));
		if(dot_prod_v < 0)angle += PI;
		angles.push_back(angle);
	}
	sort(angles.begin(), angles.end());
	float max_angle_value = angles[0];
	
	double a = 2 * PI - angles[angles.size()-1];
	if(a > max_angle_value) max_angle_value = a;
	
	for (int i = 1; i < angles.size(); ++i){
		double a = angles[i] - angles[i-1];
		if(a > max_angle_value) max_angle_value = a;
	}
	
	return max_angle_value;
}
