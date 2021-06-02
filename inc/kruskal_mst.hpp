#pragma once 
#include "utils.hpp"
#include <iterator>
#include <set>
  

typedef  std::vector< std::pair<float, std::pair<int, int>> > iTriple;
  
struct Graph{
	iTriple edges;
	iTriple edges_kruskal;
	float mst_wt;
	//add an edge
	void addEdge(int u, int v, float w);
  
	//MST algorithm
	struct DisjointSets kruskalMST(int n);
};

// To represent Disjoint Sets
struct DisjointSets{
    std::vector<int> parent, rnk;
	int n;
    // Constructor.
    DisjointSets(int n);

    // Find the parent of a node 'u'
    // Path Compression
    int find(int u);
  
    // Union by rank
	void merge(int x, int y);
	
	void color_sets(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array);
};

struct DisjointSets fill_mst(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array, float min_cost, int min_neighboor_pt);
