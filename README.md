# RANSAC-Plane-Reconstruction
Plane Reconstruction from Point Cloud using RANSAC algorithm

The following algorithm uses RANSAC to reconstruct a best-fitting 3D plane from a given Point Cloud.
The algorithm randomly chooses 3 points from the Point Cloud, constructs a 3D plane, and then calculates the distances of all the other points from that plane. A point with a distance from the plane which is smaller than a specified threshold will be considered as an "inlier" for that specific plane, otherwise it will be considered as an "outlier". The plane for which there are the most "inliers", will be declared as the "best fitting" plane.
In the code, the number of iterations and distance threshold can be tuned in order to get better results.
The results are than visualized using the mpl_toolkits package.
