#pragma once 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include  <Eigen/Core>
#include <Eigen/Eigenvalues> 

class Vector_3D{
	public: Vector_3D():x(0),y(0),z(0){};
	public: Vector_3D(pcl::PointXYZRGB c):x(0),y(0),z(0){
		center = c;
	};
	public: Vector_3D(float xt, float yt, float zt){
		x=xt;
		y=yt;
		z=zt;
	};
	public: float x;
	public: float y;
	public: float z;
	public: pcl::PointXYZRGB center;
	public: float norm(){return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));};
	public: Vector_3D operator /(float cst){
		x /= cst;
		y /= cst;
		z /= cst;
		Vector_3D* n = this;
		return *n;
	};
	public: Vector_3D operator +(Vector_3D v){
		x += v.x;
		y += v.y;
		z += v.z;
		Vector_3D* n = this;
		return *n;
	};
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

Vector_3D get_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices, pcl::PointXYZRGB c);

Vector_3D get_normal_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices, pcl::PointXYZRGB c);

std::vector<std::vector<int>> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k);

std::vector<int> RadialNeighbors(pcl::PointXYZRGB search, float radius, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree);

std::vector<int> KnnNeighbors(pcl::PointXYZRGB search, int k, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree);

pcl::PointXYZRGB neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices);

pcl::PointXYZRGB neighbourhood_centroid_proj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices, Vector_3D n);

float SquaredDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
