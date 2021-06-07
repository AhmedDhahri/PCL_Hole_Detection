#include "utils.hpp"
#include "bdr_weighted_avg.hpp"
#include "bdr_angle.hpp"
#include "kruskal_mst.hpp"

std::string path = "data.bin";
int neighbourhood_size = 30;
float wt_avg_prob_thresh = 0.50;
float angle_prob_thresh = 0.95;
float total_cost = 1.0;
int min_union_in_neighbourhood = 2;
bool dump_load;

int main(int argc, char *argv[]){
	
	if((argv[1][0] == '-') && (argv[1][1] == 'l')){
		std::cout<<"Border points array data will be loaded from hard drive."<<endl;
		dump_load = false;
	}
	if((argv[1][0] == '-') && (argv[1][1] == 'd')){
		std::cout<<"Border points array data will be dumped into hard drive."<<endl;
		dump_load = true;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("model.pcd", *cloud) == -1){
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	return (-1);
	}
	Array_saver as(path);
	float time;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = transform_rgb(cloud);
	auto start_time = std::chrono::high_resolution_clock::now();
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
	std::vector<std::pair<int, std::vector<int>>> a = get_set_points_array(ds, pi_array, cloud_rgb);
	//-----------------------------------------------------------------------------
	
	pcl::visualization::PCLVisualizer viewer("PCL TEST");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
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

