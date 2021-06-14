#pragma once 
#include "uqtr_zone_coverage_evaluation/utils.hpp"
#include <iterator>
#include <set>
  

typedef  std::vector< std::pair<float, std::pair<int, int>> > iTriple;
  
struct Graph{
	iTriple edges;
	
	int mst_edges;
	float mst_wt;
	//add an edge
	void addEdge(int u, int v, float w);
  
	//MST algorithm
	struct DisjointSets kruskalMST(int n, std::vector<Point_info>& pi_array);
};

// To represent Disjoint Sets
struct DisjointSets{
    std::vector<int> parent, rnk;
	std::set<int, std::greater<int>> parent_set;
	std::unordered_map<int, std::vector<int>> set_points;
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

int assign_children(int set, std::vector<Point_info>& pi_array, int prev_set=-1);

std::vector<int> get_chain(int set, std::vector<Point_info>& pi_array, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

std::vector<std::pair<int, std::vector<int>>> get_set_points_array(DisjointSets ds, std::vector<Point_info>& pi_array, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
