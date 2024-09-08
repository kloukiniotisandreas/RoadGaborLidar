#include "RoadDetection.hpp"

void pcl_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &final) {

	float resolution = 0.5f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	octree.leaf_begin();
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_center(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);

	for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr leaf_final(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> lef_inliers;
		it.getLeafContainer().getPointIndices(lef_inliers);
		pcl::copyPointCloud(*cloud, lef_inliers, *leaf_final);

		if (leaf_final->points.size() >= 5) {
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		    pcl::SACSegmentation<pcl::PointXYZ> seg;
			seg.setOptimizeCoefficients(false);
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(2);

			seg.setInputCloud(leaf_final);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() > 3) {

			float a = coefficients->values[0];
			float b = coefficients->values[1];
			float c = coefficients->values[2];
			float d = coefficients->values[3];
			float par = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

			float cx = 0;
			float cy = 0;
			float cz = 0;
			float di = 0;
			for (std::size_t i = 0; i < leaf_final->points.size(); ++i) {


				cx += leaf_final->points[i].x;
				cy += leaf_final->points[i].y;
				cz += leaf_final->points[i].z;

				di += abs(a*cx + b * cy + c * cz + d);

			}

			if (cz / leaf_final->points.size() < 1.4) {
				cloud_center->points.resize(cloud_center->size() + 1);
				normals->points.resize(normals->size() + 1);

				cloud_center->points[cloud_center->size() - 1].x = cx / leaf_final->points.size();
				cloud_center->points[cloud_center->size() - 1].y = cy / leaf_final->points.size();
				cloud_center->points[cloud_center->size() - 1].z = cz / leaf_final->points.size();

				normals->points[normals->size() - 1].normal_x = a / par;
				normals->points[normals->size() - 1].normal_y = b / par;
				normals->points[normals->size() - 1].normal_z = c / par;
							}
			}
		}

	}

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(50);
	reg.setInputCloud(cloud_center);

	reg.setInputNormals(normals);

	reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	int max_size = -228360;
	int max_thes;


	for (int i = 0; i < clusters.size(); i++) {

		int meg_clus = clusters[i].indices.size();
		if (max_size < meg_clus) {
			max_thes = i;
			max_size = clusters[i].indices.size();
		}

	}


	pcl::PointCloud<pcl::PointXYZ> cloud_a , cloud_c;
	pcl::copyPointCloud(*cloud_center, clusters[max_thes].indices, cloud_a);
	cloud_c = cloud_a;

	pcl::PointCloud<pcl::PointXYZ>::Ptr patwma_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud_c));
	std::vector<int> inliers;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(patwma_cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.25);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud(*patwma_cloud, inliers, *final);
	return;




}
