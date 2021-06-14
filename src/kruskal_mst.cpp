#include "uqtr_zone_coverage_evaluation/kruskal_mst.hpp"

struct DisjointSets fill_mst(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Point_info>& pi_array, float min_cost, int min_neighboor_pt){
	std::cout<<"Graph tree building..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	int border_pts = 0;
	struct Graph g;
	
	for(Point_info& pi : pi_array){
		pi.child_index = -1;
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

	std::cout<<"\tTotal graph vertecies: "<<border_pts<<std::endl;
	std::cout<<"\tTotal graph edges: "<<g.edges.size()<<std::endl;
	struct DisjointSets ds = g.kruskalMST(pi_array.size(), pi_array);
	std::cout<<"\tGraph weight: "<<g.mst_wt<<std::endl;
	std::cout<<"\tKruskal MST edges: "<<g.mst_edges<<std::endl;
	std::cout<<"\tSets number: "<<ds.parent_set.size()<<std::endl;
	
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Graph tree building finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	
	
	return ds;
}

struct DisjointSets Graph::kruskalMST(int n, std::vector<Point_info>& pi_array){
	// Initialize result
     mst_wt = 0;
     mst_edges = 0;
     
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
			mst_edges++;
			
			
			pi_array[u].kruskal_neighbours.push_back(v);
			pi_array[v].kruskal_neighbours.push_back(u);
		}
	}
	
	for(int x : ds.parent){
		int p = ds.find(x);
		if(p != x)
			ds.parent_set.insert(p);
	}
	    
	
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
	std::set<int, std::greater<int>>::iterator itr;
	std::vector<Vector_3D> color_array;
	
	color_array.push_back(Vector_3D(255, 0, 0));
	color_array.push_back(Vector_3D(0, 255, 0));
	color_array.push_back(Vector_3D(0, 0, 255));
	color_array.push_back(Vector_3D(255, 0, 255));
	color_array.push_back(Vector_3D(255, 255, 0));
	color_array.push_back(Vector_3D(0, 255, 255));
	color_array.push_back(Vector_3D(255, 255, 255));
	
	for(itr=parent_set.begin();itr!=parent_set.end();itr++){
		std::vector<int> vec;
		set_points[*itr] = vec;
	}
	
	for(Point_info& pi : pi_array){
		int s = find(pi.index);
		int i=0;
		for(itr=parent_set.begin();itr!=parent_set.end();itr++){
			if(s == *itr){
				set_points[s].push_back(pi.index);
				(*cloud)[pi.index].r = color_array[i].x;
				(*cloud)[pi.index].g = color_array[i].y;
				(*cloud)[pi.index].b = color_array[i].z;
			}
			i++;
		}
	}
	
	std::set<int, std::greater<int> >::iterator it;
    for (it = parent_set.begin(); it != parent_set.end(); it++)
        std::cout<<"\t Set "<<*it<<" contains "<<set_points[*it].size()<<" points"<<std::endl;
    std::cout << std::endl;
}

int assign_children(int set, std::vector<Point_info>& pi_array, int prev_set){
	
	if(pi_array[set].kruskal_neighbours.size() > 2){
		std::vector<std::pair<int, int>> vect;
		for(int i=0;i<pi_array[set].kruskal_neighbours.size();i++){
			if(pi_array[set].kruskal_neighbours[i] != prev_set){
				vect.push_back(std::make_pair(assign_children(pi_array[set].kruskal_neighbours[i], pi_array, set), 
												pi_array[set].kruskal_neighbours[i]));
			}
		}
		sort(vect.begin(), vect.end());
		pi_array[set].child_index = vect[vect.size()-1].second;
		return 1 + vect[vect.size()-1].first;
	}
	else if(pi_array[set].kruskal_neighbours.size() < 2)
		return 1;
	else if((pi_array[set].kruskal_neighbours[0] != prev_set) &&
			(pi_array[set].kruskal_neighbours[1] == prev_set)){ // pi_array[set].kruskal_neighbours.size() == 2
		pi_array[set].child_index = pi_array[set].kruskal_neighbours[0];
		return 1 + assign_children(pi_array[set].kruskal_neighbours[0], pi_array, set);
	}
	else if((pi_array[set].kruskal_neighbours[1] != prev_set) &&
			(pi_array[set].kruskal_neighbours[0] == prev_set)){ // pi_array[set].kruskal_neighbours.size() == 2
		pi_array[set].child_index = pi_array[set].kruskal_neighbours[1];
		return 1 + assign_children(pi_array[set].kruskal_neighbours[1], pi_array, set);
	}
	else{//maybe both do not equal to prev_set
		int v1 = assign_children(pi_array[set].kruskal_neighbours[0], pi_array, set);
		int v2 = assign_children(pi_array[set].kruskal_neighbours[1], pi_array, set);
		if(v1 > v2){
			pi_array[set].child_index = pi_array[set].kruskal_neighbours[0];
			return 1 + v1;
		}
		else{
			pi_array[set].child_index = pi_array[set].kruskal_neighbours[1];
			return 1 + v2;
		}
	}
}

std::vector<int> get_chain(int set, std::vector<Point_info>& pi_array, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	
	std::vector<int> chain;
	std::vector<int> branches;
	chain.push_back(set);
	
	
	
	for(int b : pi_array[set].kruskal_neighbours)
		branches.push_back(b);
		
	pi_array[set].kruskal_neighbours.clear();
	
	for(int b : branches)
		assign_children(b, pi_array);
	
	int iteration = 0;
	int p_index = pi_array[branches[0]].child_index;
	while(iteration < pi_array.size()){
		if(p_index != -1)
			chain.push_back(p_index);
		else
			break;
		p_index = pi_array[p_index].child_index;
		iteration++;
	}
	
	if(branches.size() < 2) return chain;
	iteration = 0;
	p_index = pi_array[branches[1]].child_index;
	while(iteration < pi_array.size()){
		if(p_index != -1)
			chain.insert(chain.begin(), p_index);
		else
			break;
		p_index = pi_array[p_index].child_index;
		iteration++;
	}
	
	return chain;
}

std::vector<std::pair<int, std::vector<int>>> get_set_points_array(DisjointSets ds, std::vector<Point_info>& pi_array, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	std::cout<<"Border point chain extraction..."<<std::endl;
	auto start_time = std::chrono::high_resolution_clock::now();
	
	std::vector<std::pair<int, std::vector<int>>> border_chains;
	
	std::set<int, std::greater<int>>::iterator it;
	for(it=ds.parent_set.begin();it!=ds.parent_set.end();it++){
		
		std::vector<int> chain = get_chain(*it, pi_array, cloud);
		border_chains.push_back(std::make_pair(*it, chain));
	}
	
	ds.color_sets(cloud, pi_array);
	
	float time = ((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now()-start_time)).count();
	std::cout<<"Border point chain extraction finished in: "<<std::endl<<"---------> "<<time<<" s"<<std::endl;
	return border_chains;
}
