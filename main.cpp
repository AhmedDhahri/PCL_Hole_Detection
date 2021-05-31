#include "utils.hpp"
#include "bdr_weighted_avg.hpp"
#include "bdr_angle.hpp"


int main (){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("model.pcd", *cloud) == -1){
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	return (-1);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = transform_rgb(cloud);
	
	
	
	auto start_time = std::chrono::high_resolution_clock::now();
	
	std::vector<Point_info> neighborhood = SymNeighbors(cloud_rgb, 20);
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<time<<std::endl;
	
	sym_weighted_avg(cloud_rgb, neighborhood, 0.5);//---------------------------//
	time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<time<<std::endl;
	
	sym_angle(cloud_rgb, neighborhood, 0.95);//---------------------------//
	time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<time<<std::endl;
	
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

