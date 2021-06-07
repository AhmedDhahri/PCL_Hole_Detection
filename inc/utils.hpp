#pragma once 
#include <bits/stdc++.h>
#include <chrono>
#include <fstream>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> 

#define DEBUG


class Vector_3D{
	public: Vector_3D();
	public: Vector_3D(float xt, float yt, float zt);
	
	public: float x;
	public: float y;
	public: float z;
	
	public: float norm();
	public: Vector_3D operator /(float cst);
};
typedef struct{
	float s_density;
	Vector_3D p_centroid;
}Centroid_Info;

typedef struct{
		int index;
		float density;
		float wtavg_prob;
		float angle_prob;
		bool bdr;
		bool wa;
		bool ag;
		std::vector<int> neighbourhood;
		std::vector<int> included_pts;
		std::vector<int> kruskal_neighbours;
		int child_index;
		Vector_3D normal;
}Point_info;
	 
class Array_saver{
	
	private: std::string path;
	
	public: Array_saver(std::string path);
	public: void dump(std::vector<Point_info> pi_array);
	public: void load(std::vector<Point_info>& pi_array);	
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

std::vector<Point_info> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k);

Centroid_Info neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices);

Centroid_Info neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n);
