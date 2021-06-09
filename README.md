# PCL_Hole_Detection
Detecting boundaries in 3D PCL mapping and compute their respective surfaces.

**Compilation:**


    -- cd build

    -- sh compile.sh



**Execution:**


     -- ./test_pcl -d: Do all computation, while saving the points' extra informations (neighboorhoods, mapping density, border criterion costs, ...) in a binary file.
 
     -- ./test_pcl -l: Load the informations from the saved binary file and proceed computation.




**Parameters:**


- neighbourhood_size: the size of the knn neighbourhood to initiate the symmetrical neighbourhood points.

- wt_avg_prob_thresh: the weghted average criteria minimum probability to generate a detection of a border point according to this criteria.

- angle_prob_thresh: the angle criteria minimum probability to generate a detection of a border point according to this criteria.

- total_cost: the minimum cost of the combonation of the 2 critereon to generate a secondary detection.

- min_union_in_neighbourhood: the minimum primary detection border points, in the neighbourhood, for a secondary detection point, to be selected in the graph.

- radius: radius of the point neighbourhood to calculate the mapping surface.

- resolution: resolution of the mapping.

- lambda: factor of the correction term in the cost of redundancy to compensate the effect of the partially empty neighbourhoods.
