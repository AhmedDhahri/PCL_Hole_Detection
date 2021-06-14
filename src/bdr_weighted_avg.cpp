#include "uqtr_zone_coverage_evaluation/bdr_weighted_avg.hpp"



void sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array, float tolerance){
	std::cout<<"Weighted average critereon..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	
	for (int i = 0; i < pi_array.size(); ++i){
		Centroid_Info cd = neighbourhood_centroid(cloud, pi_array[i].neighbourhood);
		pi_array[i].density = cd.s_density;
		
		pi_array[i].wtavg_prob = std::sqrt(cd.p_centroid.x*cd.p_centroid.x + 
											cd.p_centroid.y*cd.p_centroid.y + 
											cd.p_centroid.z*cd.p_centroid.z)/cd.s_density;
		
		if (pi_array[i].wtavg_prob > tolerance)
			pi_array[i].wa = true;
		else
			pi_array[i].wa = false;
			
	}float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Weighted average critereon finished clearly in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
}

