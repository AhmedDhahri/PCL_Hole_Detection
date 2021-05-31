#pragma once 
#include <bits/stdc++.h>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> 

class Vector_3D{
	public: Vector_3D():x(0),y(0),z(0){};

	public: Vector_3D(float xt, float yt, float zt){
		x=xt;
		y=yt;
		z=zt;
	};
	public: float x;
	public: float y;
	public: float z;
	
	public: float norm(){return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));};
	public: Vector_3D operator /(float cst){
		x /= cst;
		y /= cst;
		z /= cst;
		Vector_3D* n = this;
		return *n;
	};
};
typedef struct{
	float s_density;
	pcl::PointXYZRGB p_centroid;
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
		Vector_3D normal;
}Point_info;
	 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

std::vector<Point_info> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k);

Centroid_Info neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices);

Centroid_Info neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n);


