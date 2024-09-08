# RoadGaborLidar

This is the repository for the paper which has writen during my thesis.
The main contribution of this paper is the use of LIDAR data to create a mask that will restrict the area of the image that could eventually be the road. More specifically, we are using LIDAR data to detect the points of the ground. By contracting an Octree, we find the best-fitting plane of each leaf and by performing clustering we estimate the ground. Next, we are mapping the points of the road to the image to create a mask for the image processing step. We extract the texture orientation using Gabor Filter and thereinafter the vanishing point. The experimental results demonstrate that this training-free approach can detect horizon and vanishing point very accurately and robustly, while achieving promising performance.

# Dependencies

1) Opencv
2) PCL
3) VVRFramework


# Future Steps
1) Code refactoring
2) Use PCL exclusively
