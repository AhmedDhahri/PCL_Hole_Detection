#include "bdr_weighted_avg.hpp"



void sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info> pi_array, float tolerance){

	
	for (int i = 0; i < pi_array.size(); ++i){
		pcl::PointXYZRGB p = (*cloud)[i];
		Centroid_Info cd = neighbourhood_centroid(cloud, pi_array[i].neighbourhood);
		pcl::PointXYZRGB c = cd.p_centroid;
		
		pi_array[i].wtavg_prob = std::sqrt(c.x*c.x + c.y*c.y + c.z*c.z)/cd.s_density;
		
		if (pi_array[i].wtavg_prob > tolerance){
			pi_array[i].wa = true;
			cloud.get()->points[i].r = 255;
			cloud.get()->points[i].g = 99;
			cloud.get()->points[i].b = 71;
		}
		else
			pi_array[i].wa = false;
			
	}
}

