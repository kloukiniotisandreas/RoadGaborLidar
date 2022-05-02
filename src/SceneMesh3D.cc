#include "SceneMesh3D.hpp"
#include "RoadDetection.hpp"
#include "Thread_Image.hpp"


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <MathGeoLib.h>
#include <C2DLine.h>
#include <C2DPoint.h>
using namespace pcl;

 

typedef std::pair<int, double> myPair45;
 
void Kitti_dataset::load_pathFiles(std::string path_lidar,std::string subFolderPath,int FrameNumber,std::vector<std::string> &files) {
	std::vector<std::string> pathB;

    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
	boost::filesystem::path p(path_lidar + subFolderPath);
 
    std::vector<boost::filesystem::path> v;
    std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));

    // Sort, since directory iteration
    // Is not ordered on some file systems
    std::sort(v.begin(), v.end());
    // Append them
    for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end( v.begin() + FrameNumber); it != it_end; ++it) {
        files.push_back((*it).c_str());
    }
  
}

struct compare45
{
	bool operator()(myPair45& l, myPair45& r)
	{
		return l.second > r.second;
	}
}cmp45;

const std::size_t N = 4; // or constexpr instead of const if your compiler supports it
C2DPoint  shmeiotomis(C2DLine line1, C2DLine line2) {

	C2DPoint end2 = line2.GetPointTo();
	C2DPoint end1 = line1.GetPointTo();
	C2DPoint start2 = line2.GetPointFrom();
	C2DPoint start1 = line1.GetPointFrom();
	double x, y;
	double l1 = (start1.y - end1.y) / (start1.x - end1.x);
	double l2 = (start2.y - end2.y) / (start2.x - end2.x);
	if (start1.x == end1.x) {
		x = end1.x;
		y = l2 * x + (end2.y - l2 * end2.x);
		return C2DPoint(x, y);
	}
	else if (start2.x == end2.x) {
		x = end2.x;
		y = l1 * x + (end1.y - l1 * end1.x);
		return C2DPoint(x, y);
	}

	x = (-end1.y + l1 * end1.x + end2.y - l2 * end2.x) / (l1 - l2);
	y = end2.y + l2 * (x - end2.x);

	return C2DPoint(x, y);
}

void mult(std::vector<std::vector<float>>  A, std::vector<std::vector<float>> B, std::vector<std::vector<float>> &C) {

	for (int i = 0; i < A.size(); i++) {
		std::vector<float> akuro;
		for (int j = 0; j < B[0].size(); j++) {
			float num = 0;
			for (int k = 0; k < A[0].size(); k++) {
				num += A[i][k] * B[k][j];
			}
			akuro.push_back(num);

		}
		C.push_back(akuro);
	}


}
 
 
cv::Mat result_eik_patwma_02;

void antistixeia_simeiwn_t(pcl::PointCloud<pcl::PointXYZ>::Ptr final, int prwto, int deutero ,std::vector<std::vector<float>> P, std::vector<std::vector<float>> R, std::vector<std::vector<float>> T_velo,int cols,int rows) {

	for (int i = prwto; i < deutero; i++) {
		
		std::vector<std::vector<float>> temp(4);
		
		temp[0].push_back(final->points[i].x);
		temp[1].push_back(final->points[i].y);
		temp[2].push_back(final->points[i].z);
		temp[3].push_back(1);

		std::vector<std::vector<float>> result, result2, resultt;
		mult(P, R, result);
		mult(result, T_velo, result2);
		mult(result2, temp, resultt);
 
		int u = int(resultt[0][0] / resultt[2][0]);
		int v = int(resultt[1][0] / resultt[2][0]);

		if ((u >=  cols || u <= 0 || v >=  rows || v <= 0)) {}
		else {
			cv::circle(  result_eik_patwma_02  , cv::Point(u, v), 10, cv::Scalar(0, 0, 255), cv::FILLED, 8);
		}

	}

}
void map_lidar_points_to_image_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr final, cv::Mat irt65, cv::Mat &result_eik_koukides) {
 
	result_eik_patwma_02 = irt65.clone();
  
	std::vector<std::vector<float>> P(3);
	P[0].push_back(7.215377e+02); P[0].push_back(0.000000e+00); P[0].push_back(6.095593e+02); P[0].push_back(4.485728e+01);
	P[1].push_back(0.000000e+00); P[1].push_back(7.215377e+02); P[1].push_back(1.728540e+02); P[1].push_back(2.163791e-01);
	P[2].push_back(0.000000e+00); P[2].push_back(0.000000e+00); P[2].push_back(1.000000e+00); P[2].push_back(2.745884e-03);
	 
	std::vector<std::vector<float>> R(4); 
	R[0].push_back(9.999239e-01);  R[0].push_back(9.837760e-03);  R[0].push_back(-7.445048e-03);    R[0].push_back(0);
	R[1].push_back(-9.869795e-03); R[1].push_back(9.999421e-01);  R[1].push_back(-4.278459e-03);    R[1].push_back(0);
	R[2].push_back(7.402527e-03);  R[2].push_back(4.351614e-03);  R[2].push_back(9.999631e-01);   R[2].push_back(0);
	R[3].push_back(0); R[3].push_back(0); R[3].push_back(0); R[3].push_back(1);
 
	std::vector<std::vector<float>> T_velo(4); 
	T_velo[0].push_back(7.533745e-03); T_velo[0].push_back(-9.999714e-01); T_velo[0].push_back(-6.166020e-04); T_velo[0].push_back(-4.069766e-03);
	T_velo[1].push_back(1.480249e-02); T_velo[1].push_back(7.280733e-04);  T_velo[1].push_back(-9.998902e-01); T_velo[1].push_back(-7.631618e-02);
	T_velo[2].push_back(9.998621e-01); T_velo[2].push_back(7.523790e-03);  T_velo[2].push_back(1.480755e-02);  T_velo[2].push_back(-2.717806e-01);
	T_velo[3].push_back(0); T_velo[3].push_back(0); T_velo[3].push_back(0); T_velo[3].push_back(1);
 
	cv::Mat img_rgb;
	cv::cvtColor(irt65, img_rgb, cv::COLOR_BGR2RGB);
 
	int bima = final->size() / 4;
	std::thread thread[4];

	int prwto = 0;
	int deutero = bima;
	thread[0] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R, T_velo, irt65.cols, irt65.rows);
	prwto = deutero; deutero = deutero + bima;
	thread[1] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R, T_velo, irt65.cols, irt65.rows);
	prwto = deutero; deutero = deutero + bima;
	thread[2] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R, T_velo, irt65.cols, irt65.rows);
	prwto = deutero; deutero = final->size();
	thread[3] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R, T_velo, irt65.cols, irt65.rows);
 
	for (int i = 0; i < 4; ++i) {
		thread[i].join();
	}
	
	return;
	 
}
void load_lidar_points(std::string onoma_bin,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
	std::vector<std::vector<float>> lidar_a;
	int  num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));
 
	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
 
	// load point cloud
	FILE *stream;
 
	stream = fopen((onoma_bin).c_str(), "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	for (int i = 0; i < num; i++) {
 
		lidar_a.resize(lidar_a.size() + 1);
 
		lidar_a[i].push_back(*px);
		lidar_a[i].push_back(*py);
		lidar_a[i].push_back(*pz);
		lidar_a[i].push_back(*pr);
		px += 4; py += 4; pz += 4; pr += 4;

	}
	fclose(stream);
 
	for (int i = 0; i < lidar_a.size(); i++) {
		
		if (lidar_a[i][0] >= -1 && lidar_a[i][2] < -1.2) {
			cloud->points.push_back(pcl::PointXYZ(lidar_a[i][0], lidar_a[i][1], lidar_a[i][2]));
		 
		}
	 

	}
}

//////////////////////////////////////////////////////////////////////
// std::vector<std::vector<float>> simeia_dromou;
// std::vector<std::vector<float>> simeia_dromou_patwma;
float find_mena_var_colour(cv::Mat image_aa, C2DPoint startp, C2DPoint endp, vec& mean1, vec& mean2, cv::Mat orientation) {

	int bima = 15;
	cv::Mat perioxi2;
	cv::Mat black_imag(image_aa.rows, image_aa.cols, CV_8UC3, cv::Scalar(0, 0, 0));

	C2DLine lin = C2DLine(startp, endp);
	C2DLine lin1 = C2DLine(C2DPoint(startp.x + bima, startp.y), C2DPoint(endp.x + bima, endp.y));
	C2DLine lin2 = C2DLine(C2DPoint(startp.x - bima, startp.y), C2DPoint(endp.x - bima, endp.y));

	float paxos = 2 * lin.Distance(lin1);

	line(black_imag, cv::Point(startp.x + bima, startp.y), cv::Point(endp.x + bima, endp.y), cv::Scalar(0, 0, 255), paxos, 8);
	line(black_imag, cv::Point(startp.x - bima, startp.y), cv::Point(endp.x - bima, endp.y), cv::Scalar(255, 0, 255), paxos, 8);
 
 
	mean1 = vec(0, 0, 0);
	mean2 = vec(0, 0, 0);
	vec  var = vec(0, 0, 0);
	int arithmos_pixel1 = 0;
	int arithmos_pixel2 = 0;
 
	for (int k = 0; k < black_imag.cols; k++) {
		for (int l = 0; l < black_imag.rows; l++) {
			cv::Vec3b color2 = black_imag.at<cv::Vec3b>(l, k);
			if (color2[0] == 0 && color2[1] == 0 && color2[2] == 255 && orientation.at<double>(l, k) != 228360) {
				cv::Vec3b color3;
				color3 = image_aa.at<cv::Vec3b>(l, k);
				mean1.x = mean1.x + color3[0];
				mean1.y = mean1.y + color3[1];
				mean1.z = mean1.z + color3[2];
				arithmos_pixel1++;
			}
			else if (color2[0] == 255 && color2[1] == 0 && color2[2] == 255 && orientation.at<double>(l, k) != 228360) {
				cv::Vec3b color4;
				color4 = image_aa.at<cv::Vec3b>(l, k);
				mean2.x = mean2.x + color4[0];
				mean2.y = mean2.y + color4[1];
				mean2.z = mean2.z + color4[2];
				arithmos_pixel2++;
			}
		}
	}

	mean1.x = mean1.x / float(arithmos_pixel1);
	mean1.y = mean1.y / float(arithmos_pixel1);
	mean1.z = mean1.z / float(arithmos_pixel1);

	mean2.x = mean2.x / float(arithmos_pixel2);
	mean2.y = mean2.y / float(arithmos_pixel2);
	mean2.z = mean2.z / float(arithmos_pixel2);
 
	arithmos_pixel1 = 0;
	arithmos_pixel2 = 0;

 
	vec  var1 = vec(0, 0, 0);
	vec  var2 = vec(0, 0, 0);
	for (int k = 0; k < black_imag.cols; k++) {


		for (int l = 0; l < black_imag.rows; l++) {
			cv::Vec3b color2 = black_imag.at<cv::Vec3b>(l, k);
			if (color2[0] == 0 && color2[1] == 0 && color2[2] == 255 && orientation.at<double>(l, k) != 228360) {
				cv::Vec3b color3;
				color3 = image_aa.at<cv::Vec3b>(l, k);
				var1.x = var1.x + pow((color3[0] - mean1.x), 2);
				var1.y = var1.y + pow((color3[1] - mean1.y), 2);
				var1.z = var1.z + pow((color3[2] - mean1.z), 2);
				arithmos_pixel1++;

			}
			else if (color2[0] == 255 && color2[1] == 0 && color2[2] == 255 && orientation.at<double>(l, k) != 228360) {

				cv::Vec3b color4;
				color4 = image_aa.at<cv::Vec3b>(l, k);
				var2.x = var2.x + pow((color4[0] - mean2.x), 2);
				var2.y = var2.y + pow((color4[1] - mean2.y), 2);
				var2.z = var2.z + pow((color4[2] - mean2.z), 2);

				arithmos_pixel2++;


			}

		}
	}

	var1.x = var1.x / float(arithmos_pixel1);
	var1.y = var1.y / float(arithmos_pixel1);
	var1.z = var1.z / float(arithmos_pixel1);

	var2.x = var2.x / float(arithmos_pixel2);
	var2.y = var2.y / float(arithmos_pixel2);
	var2.z = var2.z / float(arithmos_pixel2);


	float diafx = abs(mean1.x - mean2.x) / sqrt(var1.x + var2.x);
	float diafy = abs(mean1.y - mean2.y) / sqrt(var1.y + var2.y);
	float diafz = abs(mean1.z - mean2.z) / sqrt(var1.z + var2.z);


	if (arithmos_pixel1 == 0 || arithmos_pixel2 == 0) return 0;
	if (diafx > diafy&& diafx > diafz) return diafx;
	if (diafy > diafx&& diafy > diafz) return diafy;
	if (diafz > diafx&& diafz > diafy)  return diafz;
 
}

float find_ocr_space(cv::Mat image_aa, C2DPoint startp, cv::Mat orientation, C2DLine grammi ) {

	typedef std::pair<int, double> myPair;
	struct compare2
	{
		bool operator()(myPair& l, myPair& r)
		{
			return l.second > r.second;
		}
	}cmp1;

	std::vector<myPair> pair_xanei_epafi;

	float ratio_ocr = 0;
	float simeia_gramis = 0;


	cv::Mat in, h, s;
	cv::Mat hsv_imag;
	cv::Mat new_image;

	cv::cvtColor(image_aa, hsv_imag, cv::COLOR_BGR2HSV);

	std::vector<cv::Mat> hsv_planes;
	cv::split(hsv_imag, hsv_planes);

	hsv_planes[2].copyTo(in);

	C2DLine oriz_orio = C2DLine(C2DPoint(hsv_imag.cols, hsv_imag.rows), C2DPoint(0, hsv_imag.rows));
	C2DLine katak_orio_a = C2DLine(C2DPoint(0, 0), C2DPoint(0, hsv_imag.rows));
	C2DLine katak_orio_d = C2DLine(C2DPoint(hsv_imag.cols + 1, 0), C2DPoint(hsv_imag.cols, hsv_imag.rows));

	C2DPoint oriz_simeio = shmeiotomis(oriz_orio, grammi);
	C2DPoint katak_simeio_a = shmeiotomis(katak_orio_a, grammi);
	C2DPoint katak_simeio_d = shmeiotomis(katak_orio_d, grammi);

	C2DPoint simeio_tomis;

	if (oriz_simeio.x >= 0 && oriz_simeio.x <= hsv_imag.cols)simeio_tomis = oriz_simeio;
	else if (katak_simeio_a.y < startp.y)simeio_tomis = katak_simeio_d;
	else simeio_tomis = katak_simeio_a;

	 

	C2DLine tel_line = C2DLine(C2DPoint(startp.x, startp.y), C2DPoint(simeio_tomis.x, simeio_tomis.y));

	vec mean_1 = vec(0, 0, 0);
	vec mean_2 = vec(0, 0, 0);

	float diafora_xr;
	diafora_xr = find_mena_var_colour(image_aa, C2DPoint(startp.x, startp.y), C2DPoint(simeio_tomis.x, simeio_tomis.y), mean_1, mean_2, orientation);

	vec mean = vec(0, 0, 0);
	int arithmos_pixel = 0;
 
	double mikos_g = tel_line.GetLength();
	double bima = 1 / mikos_g;
	int metritis = 0;
	bool simea = false;
 
	for (double i = 0; i < 1; i = i + bima) {

		int k = tel_line.GetPointOn(i).y;
		int l = tel_line.GetPointOn(i).x;

		if (!(k<0 || l<0 || l>orientation.cols - 1 || k>orientation.rows - 1) && orientation.at<double>(k, l) != 228360) {

			 

			double gwnia = orientation.at<double>(k, l) * pi / 180;
			C2DLine temp_t_o = C2DLine(C2DPoint(l, k), C2DPoint(l + 15 * cos(gwnia), k - 15 * sin(gwnia)));
			C2DVector l1 = grammi.vector;
			C2DVector l2 = temp_t_o.vector;

			double gwnia_l1_l2 = (l1.i * l2.i + l1.j * l2.j) / (l1.GetLength() * l2.GetLength());

		 
			if (gwnia_l1_l2 >= 0.92) {
				pair_xanei_epafi.push_back(myPair(k, 0));
				 
				ratio_ocr = ratio_ocr + 1;
				 

			}
		}

		simeia_gramis++;

	}
 
	if (simeia_gramis == 0)return 0;
	ratio_ocr = ratio_ocr / simeia_gramis;

	return ratio_ocr * diafora_xr;


}
void recalculate_v_p(C2DPoint  v_p, cv::Mat texture_oriantation_a, cv::Mat image_g_f, std::string name,cv::Mat &result) {
	
	float max_per = -228360;
	float min_per = 228360;

	cv::Mat texture_oriantation = texture_oriantation_a.clone();

	cv::Mat in;
	cv::Mat coloured_image;
	cv::cvtColor(image_g_f, in, cv::COLOR_BGR2GRAY);

	cv::Mat col_cut2 = image_g_f.clone();
	cv::Mat colour_image = image_g_f.clone();

	int max_col = v_p.y;
	int	max_row = v_p.x;
	C2DPoint van = C2DPoint(max_col, max_row);
	std::vector<C2DLine> rays;
	for (int i = 15; i < 170; i = i + 5) {

		rays.push_back(C2DLine(C2DPoint(max_col - ((col_cut2.rows - max_row) / sin(i * M_PI / 180)) * cos(i * M_PI / 180), max_row + ((col_cut2.rows - max_row) / sin(i * M_PI / 180)) * sin(i * pi / 180)), C2DPoint(max_col, max_row)));

	}

	typedef std::pair<int, double> myPair;
	typedef std::pair<C2DLine, double>  line_score_Pair;
	struct compare2
	{
		bool operator()(myPair& l, myPair& r)
		{
			return l.second > r.second;
		}
	}cmp1;

	struct compare34
	{
		bool operator()(line_score_Pair& l, line_score_Pair& r)
		{
			return l.second > r.second;
		}
	}cmp34;

	std::vector<myPair> ocr;
	std::vector<myPair> thesi_pou_xanei_epafi;

	ocr.push_back(myPair(0, 0));
	thesi_pou_xanei_epafi.push_back(myPair(0, 0));

	for (int i = 1; i < rays.size() - 1; i++) {

		ocr.push_back(myPair(i, 0));
		thesi_pou_xanei_epafi.push_back(myPair(i, 0));

	 
		int aspra;
		int xanei_epafi;
		float ocr_ratio = find_ocr_space(col_cut2, C2DPoint(max_col, max_row), texture_oriantation, rays[i] );

		if (ocr_ratio > max_per)max_per = ocr_ratio;
		if (ocr_ratio < min_per)min_per = ocr_ratio;
		ocr[ocr.size() - 1].second = ocr_ratio;
		thesi_pou_xanei_epafi[thesi_pou_xanei_epafi.size() - 1].second = xanei_epafi;

	}

	ocr.push_back(myPair(rays.size() - 1, 0));
	thesi_pou_xanei_epafi.push_back(myPair(thesi_pou_xanei_epafi.size() - 1, 0));
	sort(ocr.begin(), ocr.end(), cmp1);
	int  xan_epafi = thesi_pou_xanei_epafi[ocr[0].first].second;

	C2DVector l1 = rays[ocr[0].first].vector;
	double klisieuthias = atan(-l1.j / l1.i);
	cv::arrowedLine(col_cut2, cv::Point(0, int(xan_epafi)), cv::Point(col_cut2.cols, int(xan_epafi)), cv::Scalar(0, 220, 0), 2, 8);
	cv::arrowedLine(col_cut2, cv::Point(max_col, max_row), cv::Point(rays[ocr[0].first].GetPointOn(0.5).x, rays[ocr[0].first].GetPointOn(0.5).y), cv::Scalar(255, 255, 125), 2, 8);
	cv::arrowedLine(col_cut2, cv::Point(max_col, max_row), cv::Point(rays[ocr[0].first].GetPointOn(0.5).x, rays[ocr[0].first].GetPointOn(0.5).y), cv::Scalar(255, 0, 125), 2, 8);

	cv::circle(col_cut2, cv::Point(max_col, max_row), 5, cv::Scalar(0, 255, 0), cv::FILLED, 8);
	result = col_cut2.clone();
	return ;

}

void   CalculateDominantDeriction(Kitti_dataset dataset)
{
	
 
	great_gabor_kernels();

	for(int i=0;i< dataset.images.size();i++){
		std::cout<< i <<"\n";
		cv::Mat imageLidarProgected;

		pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::cout << "[vp_detection]: lidar loading \n"<< std::endl;
		load_lidar_points(dataset.lidars[i], cloud);
		cv::Mat arxiki_eikona = cv::imread(dataset.images[i], cv::IMREAD_COLOR);
		cv::Mat irt = cv::imread(dataset.images[i], cv::IMREAD_COLOR);
		
		pcl_segmentation(cloud, final);
		std::cout << "[vp_detection]: Ground points \n"<< std::endl;
		map_lidar_points_to_image_plane(final, arxiki_eikona , imageLidarProgected); 

  		Image_pr frame(irt, result_eik_patwma_02);

		frame.Garbor_filter();
		std::vector<std::string> result_name;
		boost::split(result_name, dataset.images[i], boost::is_any_of("\\/"));
		recalculate_v_p(C2DPoint(frame.image_vx, frame.image_vy), frame.image_texture,frame.image, result_name.back(), frame.final_image);
		cv::imshow("VP and dominantn boarder",frame.final_image);
	}
	
	return;
	
 
}
 
typedef std::pair<int, float> iPair;
typedef std::pair<std::pair<int, int>, float> iPair2;
struct compare
{
	bool operator()(iPair& l, iPair& r)
	{
		return l.second < r.second;
	}
}cmp;
struct compare2
{
	bool operator()(iPair2& l, iPair2& r)
	{
		return l.second < r.second;
	}
}cmp2;

 
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
	
	
	CalculateDominantDeriction(dataset);
	 


}	 

