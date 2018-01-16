# FoVMax
Code for maximizing the total field of view for a camera array as described in the FoV paper. This code utilizes
Erik A Johnson's (<JohnsonE@usc.edu>) Clipper2 polygon clipping library to perform polygon clipping operations. 

------------------------------------ Utility Scripts -----------------------------------------------------------

array_area.m - Calculates the FoV area based on the parameters of a camera array. 

combinePoly.m - performs the union or intersection of polygons and checks to see that the result can be described as a single simple polygon. 

FOVcone.m - Calculate the bounding rays for the viewing cone. 

FOVproject.m - Calculates FoV using the viewing cone and the stitching plane. 

greedyBox.m - Finds the optimal place for a greedy algorithm to be applied to a grid based camera array. 

isConnected.m - Calculates whether an adjacency matrix is connected using 

maxCoverage.m - Finds the optimal place for a camera to be added for a greedy algorithm applied to the trocar shaped camera array. 

polyclip_test.m - Code provided by the Clipper2 polygon clipping library 

polyclip.m - Code provided by the Clipper2 polygon clipping library 

polyout_test.m - Code provided by the Clipper2 polygon clipping library 

polyout.m - Code provided by the Clipper2 polygon clipping library.

ray_plane_intersect.m - Perform ray-plane intersection for a given ray and plane. 

rotateMat.m - Generate a rotation matrix with the given magnitude of rotation around the x,y, and z, axes. 


------------------------------------ Optimization Examples -----------------------------------------------------


FOV_naive.m - Performs a naive optimization over the trocar based array. 

FOV_symmetric.m - Simplifies optimization down to two parameters and optimizes over trocar array. 

FOV_suboptimal.m - Performs the greedy optimization technique on the trocar array. 

FOV_exhaustive.m - Performs exhaustive search to optimize over all parameters on the trocar based array. 

FOV_comprehensive.m - This script compiles all methods for optimizing array FoV. 

FOV_grid.m - Performs optimization on a grid based camera array. 

FOV_single.m - generates a single camera array and calculates the total FoV for that array. 