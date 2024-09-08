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

	std::string data_path = argv[1];
	std::string FrameNumber = argv[2];
    std::string filename = data_path.substr(data_path.find_last_of("\\/")+1);
	
	std::cout << "[kitti_loader]: Opening Dataset \"" << filename << "\""<< std::endl;
	Kitti_dataset dataset(std::stoi(FrameNumber) ,data_path);
	
	
	calculateDominantDeriction(dataset);
	 


}	 

