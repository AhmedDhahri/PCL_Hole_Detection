#include "uqtr_zone_coverage_evaluation/bdr_angle.hpp"

void sym_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array, float tolerance){
	std::cout<<"Angle critereon..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	
	for (int i = 0; i < pi_array.size(); ++i){
		pi_array[i].normal = get_normal_pca(cloud, pi_array[i].neighbourhood);
		
		
		pcl::PointXYZRGB c = (*cloud)[i];
		pi_array[i].angle_prob = max_angle(cloud, pi_array[i])/PI;
		if (pi_array[i].angle_prob > tolerance)
			pi_array[i].ag = true;
		else
			pi_array[i].ag = false;
	}
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Angle critereon finished clearly in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
}

double max_angle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point_info& pi){
	Vector_3D u(((*cloud)[pi.neighbourhood[1]].x - (*cloud)[pi.neighbourhood[0]].x),
				((*cloud)[pi.neighbourhood[1]].y - (*cloud)[pi.neighbourhood[0]].y),
				((*cloud)[pi.neighbourhood[1]].z - (*cloud)[pi.neighbourhood[0]].z));//axix
	float dot_prod_u = u.x * pi.normal.x + u.y * pi.normal.y + u.z * pi.normal.z;
	u.x = u.x - pi.normal.x * dot_prod_u;
	u.y = u.y - pi.normal.y * dot_prod_u;
	u.z = u.z - pi.normal.z * dot_prod_u;
	u = u / u.norm();
	
	Vector_3D v(pi.normal.y*u.z - pi.normal.z*u.y,
				pi.normal.z*u.x - pi.normal.x*u.z,
				pi.normal.x*u.y - pi.normal.y*u.x);//ord
	v = v / v.norm();
	std::vector<double> angles;
	for (int i = 2; i < pi.neighbourhood.size(); ++i){
					
		float dot_prod_u = 	((*cloud)[pi.neighbourhood[i]].x - (*cloud)[pi.neighbourhood[0]].x) * u.x + 
							((*cloud)[pi.neighbourhood[i]].y - (*cloud)[pi.neighbourhood[0]].y) * u.y + 
							((*cloud)[pi.neighbourhood[i]].z - (*cloud)[pi.neighbourhood[0]].z) * u.z;
							
							
		float dot_prod_v = 	((*cloud)[pi.neighbourhood[i]].x - (*cloud)[pi.neighbourhood[0]].x) * v.x + 
							((*cloud)[pi.neighbourhood[i]].y - (*cloud)[pi.neighbourhood[0]].y) * v.y + 
							((*cloud)[pi.neighbourhood[i]].z - (*cloud)[pi.neighbourhood[0]].z) * v.z;
		
		
		double angle = acos(dot_prod_u/std::sqrt(dot_prod_u*dot_prod_u + dot_prod_v*dot_prod_v));
		if(dot_prod_v < 0)angle = 2 * PI -angle;
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
