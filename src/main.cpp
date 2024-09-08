#include "SceneMesh3D.hpp"
#include "src/RoadDetection.hpp"
#include "src/Thread_Image.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>


int main(int argc, char* argv[])
{
	std::cout << "[kitti_loader]: Starting up" << std::endl;
	if(argc != 3) {
        std::cerr << "[kitti_loader]: Please give input for data path and number of frames" << std::endl;
        return EXIT_FAILURE;
    }

	std::string dataPath = argv[1];
	std::string frameNumber = argv[2];

	std::cout << "[kitti_loader]: Opening Dataset \"" << dataPath << "\""<< std::endl;

	Kitti_dataset dataset(std::stoi(frameNumber) ,dataPath);


	calculateDominantDeriction(dataset);


}
