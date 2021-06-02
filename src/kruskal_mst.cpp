#include "kruskal_mst.hpp"

struct DisjointSets fill_mst(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array, float min_cost, int min_neighboor_pt){
	std::cout<<"Graph tree building..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	int border_pts = 0;
	struct Graph g;
	
	for(Point_info& pi : pi_array){
		
		if(pi.wa && pi.ag){
#ifdef DEBUG
			(*cloud)[pi.index].r = 255;
			(*cloud)[pi.index].g = 0;
			(*cloud)[pi.index].b = 0;
#endif
			border_pts++;
			pi.bdr = true;
		}
		else if(pi.wtavg_prob + pi.angle_prob > min_cost){
			int tmp = 0;
			for(int i : pi.neighbourhood){
				if(pi_array[i].wa && pi_array[i].ag)
					tmp++;
				if(tmp >= min_neighboor_pt){
#ifdef DEBUG
					(*cloud)[pi.index].r = 0;
					(*cloud)[pi.index].g = 255;
					(*cloud)[pi.index].b = 0;
#endif
					border_pts++;
					pi.bdr = true;
					break;
				}
			}
		}
		else
			pi.bdr = false;
	}
	
	
	for(Point_info pi : pi_array){
		for(int ix : pi.neighbourhood){
			if(pi_array[ix].bdr && pi.bdr &&
					(std::find(pi.included_pts.begin(), pi.included_pts.end(), pi_array[ix].index)
												== pi.included_pts.end())){

					float x = (*cloud)[pi.index].x - (*cloud)[pi_array[ix].index].x;
					float y = (*cloud)[pi.index].y - (*cloud)[pi_array[ix].index].y;
					float z = (*cloud)[pi.index].z - (*cloud)[pi_array[ix].index].z;
					
					float w = 2 * std::sqrt(x * x + y * y + z * z) / (pi.density + pi_array[ix].density);
					g.addEdge(pi.index, pi_array[ix].index, w);
					
					
					pi.included_pts.push_back(pi_array[ix].index);
					pi_array[ix].included_pts.push_back(pi.index);
			}
		}
	}

	std::cout<<"Total graph vertecies: "<<border_pts<<std::endl;
	std::cout<<"Total graph edges: "<<g.edges.size()<<std::endl;
	struct DisjointSets ds = g.kruskalMST(pi_array.size());
	std::cout<<"Graph weight: "<<g.mst_wt<<std::endl;
	ds.color_sets(cloud, pi_array);
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Graph tree building finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	return ds;
}

struct DisjointSets Graph::kruskalMST(int n){
	// Initialize result
     mst_wt = 0;
     
	// Sort edges in increasing order on basis of cost
	sort(edges.begin(), edges.end());
  
	// Create disjoint sets
	DisjointSets ds(n);
	
	// Iterate through all sorted edges
	for (iTriple::iterator it=edges.begin(); it!=edges.end(); it++){
		
		int u = it->second.first;
		int v = it->second.second;
  
		
		int set_u = ds.find(u);
		int set_v = ds.find(v);

		// Check if the selected edge is creating
		// a cycle or not (Cycle is created if u
		// and v belong to same set)
		if (set_u != set_v){
			// Merge two sets
			ds.merge(set_u, set_v);
			// Update MST weight
			mst_wt += it->first;
			
			edges_kruskal.push_back(*it);
		}
	}
	
	std::set<int, std::greater<int> > parent_set;
	for(int x : ds.parent){
		int p = ds.find(x);
		if(p != x)
			parent_set.insert(p);
	}
	
	std::cout<<"The "<<parent_set.size()<<" set are: ";
	std::set<int, std::greater<int> >::iterator itr;
    for (itr = parent_set.begin(); itr != parent_set.end(); itr++)
        std::cout << *itr<<" ";
    std::cout << std::endl;
    std::cout<<"Kruskal MST edges: "<<edges_kruskal.size()<<std::endl;
    
	
	return ds;
}

void Graph::addEdge(int u, int v, float w){
		edges.push_back({w, {u, v}});
}
	
DisjointSets::DisjointSets(int n){
	this->n = n;

	// Initially, all vertices are in different sets and have rank 0.
	for (int i = 0; i <= n; i++){
		rnk.push_back(0);

		//every element is parent of itself
		parent.push_back(i);
	}
}

int DisjointSets::find(int u){
	/* Make the parent of the nodes in the path
	from u--> parent[u] point to parent[u] */
	if (u != parent[u])
		parent[u] = find(parent[u]);
	return parent[u];
}

void DisjointSets::merge(int x, int y){// Union by rank
	//int x = find(x), y = find(y);
  
	// Make tree with smaller height
	//a subtree of the other tree 
	if (rnk[x] > rnk[y])
		parent[y] = x;
	else // If rnk[x] <= rnk[y]
		parent[x] = y;
	
	if (rnk[x] == rnk[y])
		rnk[y]++;
}

void DisjointSets::color_sets(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array){
	//94580 84123 83412 35381 21057
	for(Point_info& pi : pi_array){
		int s = find(pi.index);
		switch(s){
			case 94580:
				(*cloud)[pi.index].r = 255;
				(*cloud)[pi.index].g = 0;
				(*cloud)[pi.index].b = 0;
				break;
			case 84123:
				(*cloud)[pi.index].r = 0;
				(*cloud)[pi.index].g = 255;
				(*cloud)[pi.index].b = 0;
				break;
			case 83412:
				(*cloud)[pi.index].r = 0;
				(*cloud)[pi.index].g = 0;
				(*cloud)[pi.index].b = 255;
				break;
			case 35381:
				(*cloud)[pi.index].r = 255;
				(*cloud)[pi.index].g = 255;
				(*cloud)[pi.index].b = 0;
				break;
			case 21057:
				(*cloud)[pi.index].r = 255;
				(*cloud)[pi.index].g = 0;
				(*cloud)[pi.index].b = 255;
				break;
			default:
				(*cloud)[pi.index].r = 150;
				(*cloud)[pi.index].g = 150;
				(*cloud)[pi.index].b = 150;
				break;
		}
	}
}
