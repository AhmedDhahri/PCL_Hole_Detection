#include "main.hpp"
#include "bdr_weighted_avg.hpp"


int main (){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("model.pcd", *cloud) == -1){
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	return (-1);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = transform_rgb(cloud);
	radius_weighted_avg(cloud_rgb);//
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
