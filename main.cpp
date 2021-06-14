#include "uqtr_zone_coverage_evaluation/utils.hpp"
#include "uqtr_zone_coverage_evaluation/bdr_weighted_avg.hpp"
#include "uqtr_zone_coverage_evaluation/bdr_angle.hpp"
#include "uqtr_zone_coverage_evaluation/kruskal_mst.hpp"
#include "uqtr_zone_coverage_evaluation/surface_calculation.hpp"


/*******************************************
std::string model_path = "sim_test.pcd";
std::string checkpoint_path = "data.bin";
int neighbourhood_size = 30;
float wt_avg_prob_thresh = 0.50;
float angle_prob_thresh = 0.95;
float total_cost = 1.0;
int min_union_in_neighbourhood = 2;

float radius = 0.02;
float resolution = 0.000001256;
float lambda = 100;
*******************************************/

std::string model_path = "sim_test.pcd";
std::string checkpoint_path = "data.bin";
int neighbourhood_size = 20;
float wt_avg_prob_thresh = 0.50;
float angle_prob_thresh = 0.9;
float total_cost = 0.9;
int min_union_in_neighbourhood = 2;

int test_k = 5;
int min_bdr = 3;
float radius = 0.5;
float resolution = 0.05;


bool dump_load;


int main(int argc, char *argv[]){
	//-----------------------------------------------------------------------------
	if((argv[1][0] == '-') && (argv[1][1] == 'l')){
		std::cout<<"Border points array data will be loaded from hard drive."<<endl;
		dump_load = false;
	}
	if((argv[1][0] == '-') && (argv[1][1] == 'd')){
		std::cout<<"Border points array data will be dumped into hard drive."<<endl;
		dump_load = true;
	}
	Array_saver as(checkpoint_path);
	//-----------------------------------------------------------------------------
	pcl::PointCloud<POINT_TYPE>::Ptr cloud (new pcl::PointCloud<POINT_TYPE>);
	if (pcl::io::loadPCDFile<POINT_TYPE> (model_path, *cloud) == -1){
		PCL_ERROR ("Couldn't read file .pcd \n");
		return (-1);
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = transform_rgb(cloud);
	//-----------------------------------------------------------------------------
	std::vector<Point_info> pi_array;
	if(!dump_load)
		as.load(pi_array);
	//-----------------------------------------------------------------------------
	else{
		pi_array = SymNeighbors(cloud_rgb, neighbourhood_size);
		sym_weighted_avg(cloud_rgb, pi_array, wt_avg_prob_thresh);
		sym_angle(cloud_rgb, pi_array, angle_prob_thresh);
		as.dump(pi_array);
	}
	//-----------------------------------------------------------------------------
	struct DisjointSets ds = fill_mst(cloud_rgb, pi_array, total_cost, min_union_in_neighbourhood);
	std::vector<std::pair<int, std::vector<int>>> point_sets_array 
													= get_set_points_array(ds, pi_array, cloud_rgb);
	//-----------------------------------------------------------------------------
	rm_out_border(cloud_rgb, point_sets_array, pi_array, test_k, min_bdr);
	std::vector<std::pair<int, float>> s = get_hole_surface(cloud_rgb, point_sets_array);
	float surface = get_pcl_surface(cloud_rgb, radius, resolution);
	//-----------------------------------------------------------------------------
	pcl::visualization::PCLVisualizer viewer("PCL TEST");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.setCameraPosition(0, 14, -14, 0, 0, 1, 0, -1, 0);
	viewer.resetCamera();
	viewer.setPosition(0, 0);
	viewer.setSize(1280, 720);
	viewer.addPointCloud(cloud_rgb);

	while (!viewer.wasStopped ()){
		viewer.spinOnce ();
		pcl_sleep(0.017);
	}
	return (0);
}

