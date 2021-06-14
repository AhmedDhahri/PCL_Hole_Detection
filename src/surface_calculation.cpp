#include "uqtr_zone_coverage_evaluation/surface_calculation.hpp"

void rm_out_border(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
					std::vector<std::pair<int, std::vector<int>>>& set_points_array,
					std::vector<Point_info> pi_array,
					int k, int min_bdr){
						
	std::cout<<"Point chains sets filtering..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
						
	for(auto i = set_points_array.begin();i != set_points_array.end();i++){
		
		pcl::PointXYZRGB p = get_center(cloud, i->second);
		
		std::cout<<"\tSet "<<i->first;
		std::vector<int> indices;
		std::vector<float> sqDistance;
		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
		
		kdtree.setInputCloud(cloud);
		kdtree.nearestKSearch(p, k, indices, sqDistance);
		int cnt = 0;
		for(int index : indices){
			if(pi_array[index].bdr)cnt++;
		}
		if(cnt <= min_bdr){//test if bdr in the same set
			set_points_array.erase(i);
			i--;
			std::cout<<" removed";
		}
		std::cout<<" "<<cnt<<std::endl;
	}
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Point chains sets filtering finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
}


std::vector<std::pair<int, float>> get_hole_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<int, std::vector<int>>> set_points_array){
	
	std::cout<<"Hole surfaces calculation..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	std::vector<std::pair<int, float>> set_surfaces;
	
	for(auto p : set_points_array){
		
		Vector_3D n = get_normal_pca(cloud, p.second);
		
		pcl::PointXYZRGB center = get_center(cloud, p.second);
		Vector_3D u( (*cloud)[p.second[0]].x - center.x,
					 (*cloud)[p.second[0]].y - center.y,
					 (*cloud)[p.second[0]].z - center.z);
		u = u / u.norm();// devide 0
		if(u.norm() == 0)
			std::cerr<<"\tDivide by 0 - Hole surfaces calculation"<<std::endl;
		Vector_3D v(n.y * u.z - n.z * u.y,
					n.z * u.x - n.x * u.z,
					n.x * u.y - n.y * u.x);
					
					
		float area = 0;
		float xc_old = 0, yc_old = 0;
		bool b = true;
		
		for(int in : p.second){
			float ax = (*cloud)[in].x - center.x;
			float ay = (*cloud)[in].y - center.y;
			float az = (*cloud)[in].z - center.z;
			
			float xc =  ax * u.x + ay * u.y + az * u.z;
			float yc =  ax * v.x + ay * v.y + az * v.z;
			if(yc < 0) yc = -yc;

			
			if(b){
				b = false;
				continue;
			}
			else{
				//trapezoid area
				float width = xc - xc_old;
				if(width < 0) width = -width;
				area += 0.5 * (yc + yc_old) * width;
			}
			yc_old = yc;
			xc_old = xc;
		}
		set_surfaces.push_back(std::make_pair(p.first, area));
		std::cout<<"\tSet "<<p.first<<" Area: "<<area<<" m^2"<<std::endl;
	}
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Hole surfaces calculation finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	return set_surfaces;
}

pcl::PointXYZRGB get_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indecies){//similar one
	pcl::PointXYZRGB p;
	
	float x = 0, y = 0, z = 0;
	for(int i : indecies){
		x += (*cloud)[i].x;
		y += (*cloud)[i].y;
		z += (*cloud)[i].z;
	}
	p.x = x/indecies.size();
	p.y = y/indecies.size();
	p.z = z/indecies.size();
	return p;
}

float get_pcl_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float resolution){
	std::clog<<"Mapping surface calculation..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	float surface = 0;
	
	int reg_inner_pt = (int)std::round( radius * radius * PI / resolution);
	if(reg_inner_pt == 0) {
		std::cerr<<"\tRadius too small or resolution too big !"<<std::endl;
		exit(0);
	}
	
	std::vector<Surface_Info> si_array = radial_neighbors(cloud, radius);
	avg_distance(cloud, si_array);
	redundancy_factor(cloud, si_array, reg_inner_pt, radius);
	
	for(Surface_Info& si : si_array)
		surface += resolution * si.f_redundancy;
	
	std::cout<<"\tMapping surface: "<<surface<<" m^2"<<std::endl;
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::clog<<"Mapping surface calculation finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	return surface;
}

std::vector<Surface_Info> radial_neighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius){
	
	std::vector<Surface_Info> si_array;
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	
	for (int i = 0; i < cloud->size(); ++i){
		Surface_Info si;
		si.index = i;
		std::vector<float> sqDistance;
		kdtree.radiusSearch((*cloud)[i], radius, si.neighbourhood, sqDistance);
		si_array.push_back(si);
	}
	return si_array;
}

void avg_distance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Surface_Info>& si_array){
	
	for(Surface_Info& si : si_array){//TODO: detect small neighbourhoods
		float x = 0, y = 0, z = 0;
		for(int i=1;i<si.neighbourhood.size();i++){
			x += (*cloud)[si.neighbourhood[i]].x;
			y += (*cloud)[si.neighbourhood[i]].y;
			z += (*cloud)[si.neighbourhood[i]].z;
		}
		x = (x / (si.neighbourhood.size() - 1)) - (*cloud)[si.neighbourhood[0]].x;
		y = (y / (si.neighbourhood.size() - 1)) - (*cloud)[si.neighbourhood[0]].y;
		z = (z / (si.neighbourhood.size() - 1)) - (*cloud)[si.neighbourhood[0]].z;
		si.avg_pt_dist = std::sqrt(x * x + y * y + z * z);
	}
}

void redundancy_factor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
					std::vector<Surface_Info>& si_array, int reg_inner_pt, float radius){
	float lambda = 1/ radius;
	for(Surface_Info& si : si_array)
		si.f_redundancy = reg_inner_pt / (si.neighbourhood.size() * (1 + lambda * si.avg_pt_dist));
		//lambda * si.avg_pt_dist: this term is used to correct possible 
		//partially empty neighbourhood (ex: border points) to avoid 
		//redundancy_factor exceeding 1.0 (not logical)
}
