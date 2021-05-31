#include "kruskal_mst.hpp"

float fill_mst(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> bdr_wavg, std::vector<int> bdr_angl, std::vector<int> neighborhood){
	
	
	
	return 0.0;
}

float Graph::kruskalMST(int n){
    float mst_wt = 0; // Initialize result
  
    // Sort edges in increasing order on basis of cost
    sort(edges.begin(), edges.end());
  
    // Create disjoint sets
    DisjointSets ds(n);
  
    // Iterate through all sorted edges
    iTriple::iterator it;
    for (it=edges.begin(); it!=edges.end(); it++)
    {
        int u = it->second.first;
        int v = it->second.second;
  
        int set_u = ds.find(u);
        int set_v = ds.find(v);
  
        // Check if the selected edge is creating
        // a cycle or not (Cycle is created if u
        // and v belong to same set)
        if (set_u != set_v)
        {
  
            // Update MST weight
            mst_wt += it->first;
  
            // Merge two sets
            ds.merge(set_u, set_v);
        }
    }
  
    return mst_wt;
}
