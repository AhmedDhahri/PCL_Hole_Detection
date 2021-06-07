# PCL_Hole_Detection
Detecting boundaries in 3D PCL mapping and compute their respective surfaces.

*Compilation:*
- cd build
- sh compile.sh

*Execution:*
 - ./test_pcl -d: Do all computation, while saving the points' extra informations (neighboorhoods, mapping density, border criterion costs, ...) in a binary file.
 - ./test_pcl -l: Load the informations from the saved binary file and proceed computation.

