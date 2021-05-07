#pragma once 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

std::vector<std::vector<int>> SymNeighbors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int k);

std::vector<int> RadialNeighbors(pcl::PointXYZRGB search, float radius, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree);

std::vector<int> KnnNeighbors(pcl::PointXYZRGB search, int k, const pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree);

pcl::PointXYZRGB neighbourhood_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const std::vector<int>& indices);

float SquaredDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);

void radius_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void knn_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void sym_weighted_avg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
