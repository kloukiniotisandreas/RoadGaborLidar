
#include "Thread_Image.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/point_types.h"
#include "pcl/common/geometry.h"
using namespace std;
#define M_PI       3.14159265358979323846   // pi
std::vector<std::vector<cv::Mat>> gabor_kernels_r;
std::vector<std::vector<cv::Mat>> gabor_kernels_i;
cv::Mat  mgetGaborKernel(cv::Size ksize, double sigma, double theta, double lambd, double gamma, double psi)
{
 	double sigma_x = sigma;
 	double sigma_y = sigma / gamma;
 	int nstds = 3;
 	int xmin, xmax, ymin, ymax;
 	double c = cos(theta), s = sin(theta);

 	if (ksize.width > 0)
 		xmax = ksize.width / 2;
 	else
 		xmax = cvRound(std::max(fabs(nstds*sigma_x*c), fabs(nstds*sigma_y*s)));

 	if (ksize.height > 0)
 		ymax = ksize.height / 2;
 	else
 		ymax = cvRound(std::max(fabs(nstds*sigma_x*s), fabs(nstds*sigma_y*c)));

 	xmin = -xmax;
 	ymin = -ymax;

 	int ktype = CV_64F;

 	cv::Mat kernel(ymax - ymin + 1, xmax - xmin + 1, ktype);
 	double  scale = 1;
 	double ex = -0.5 / (sigma_x*sigma_x);
 	double ey = -0.5 / (sigma_y*sigma_y);
 	//double cscale =  CV_PI * 2 / lambd;
 	double cscale = lambd;
 	double  max = -INFINITY;
 	double  min = INFINITY;
 	//scale = sigma / (sqrt(2 * CV_PI)*2.2);

 	double mean = 0;
 	int metr_mean = 0;
 	for (int y = ymin; y <= ymax; y++)
 		for (int x = xmin; x <= xmax; x++)
 		{
 			double xr = x * c + y * s;
 			double yr = -x * s + y * c;

 			double v = scale * std::exp(ex*xr*xr + ey * yr*yr)*cos(cscale*xr + psi);
 			kernel.at<double>(ymax - y, xmax - x) = v;

 			if (v > max)max = v;

 			if (v < min)min = v;


 			mean += v;
 			metr_mean++;


 		}

 	mean = mean / metr_mean;

 	cv::Mat mmean(ymax - ymin + 1, xmax - xmin + 1, ktype);
 	mmean = mean;
    cv::Mat f_kernel = kernel - mmean;
 	cv::normalize(f_kernel, f_kernel, min - mean, max - mean);

 	return f_kernel;

}

 /*
void Image_pr::find_votes_thead (int vx, int vy)
{
	pcl::PointXYZ p, vxy;

	p.z = 0; vxy.z = 0;

	int aristera = vx - image_radius;
	if (aristera < 0)aristera = 0;

	int deksia = vx + image_radius;
	if (deksia > image_cols)deksia = image_cols;

	int katw = vy + int(image_radius);
	if (katw > image_rows)katw = image_rows;

	double vote_points = 0;
	for (int l = aristera; l < deksia; l++) {
		for (int k = vy; k < katw; k++) {

			C2DLine line1 = C2DLine(C2DPoint(vx,vy), C2DPoint(l, k));
			if (image_texture.at<double>(k, l) != 228360 && line1.GetLength() <= image_radius && l != vx && k != vy) {

				float gwnia = image_texture.at<double>(k, l) * pi / 180;


				p.x   =   l; p.y = k;
				vxy.x = vx; vxy.y = vy;
				float dist_p_vxy = pcl::geometry::distance(p, vxy);

				double add = 0;
				double cos_theta;
				if (p.x <= vx) cos_theta = asin((p.y - vy) / dist_p_vxy);
				else cos_theta = pi - asin((p.y - vy) / dist_p_vxy);

				double gamma = abs((gwnia - cos_theta) * 180 / pi);
				if (gamma < 5 / (1 + 2 * dist_p_vxy)) {

					vote_points += 1 / (1 + pow(gamma * dist_p_vxy, 2));
				}
				else {
					vote_points += 0;
				}

			}

		}
	}
	mtx.lock();
	if (vote_points > score) {
		score = vote_points;
		image_vx = vy;
		image_vy = vx;

	}
	mtx.unlock();


}
*/
void Image_pr::find_votes_thead_l(  int vy1, int vy2)
{
	double vote_points, cos_theta, aristera, deksia, katw;
	float gwnia;
	pcl::PointXY  p, vxy;

	for (int vy = vy1; vy < vy2; vy = vy + 4) {
		for (int vx = 0; vx < image_cols; vx = vx + 4) {

			  aristera = vx - image_radius;
			if (aristera < 0)aristera = 0;

			  deksia = vx + image_radius;
			if (deksia > image_cols)deksia = image_cols;

			  katw = vy +   image_radius ;
			if (katw > image_rows)katw = image_rows;

			  vote_points = 0;

			for (int l = aristera; l < deksia; l++) {
				for (int k = vy; k < katw; k++) {


					p.x = l; p.y = k;
					vxy.x = vx; vxy.y = vy;
					float dist_p_vxy = sqrt(pow(l - vx, 2) + pow(k - vy, 2));

					if (image_texture.at<double>(k, l) != 228360 && dist_p_vxy <= image_radius && l != vx && k != vy) {

						 gwnia = image_texture.at<double>(k, l) * M_PI / 180;

						if (p.x <= vx) cos_theta = asin((p.y - vy) / dist_p_vxy);
						else cos_theta = M_PI - asin((p.y - vy) / dist_p_vxy);

						double gamma = abs((gwnia - cos_theta) * 180 / M_PI);
						if (gamma < 5 / (1 + 2 * dist_p_vxy)) {

							vote_points += 1 / (1 + pow(gamma * dist_p_vxy, 2));
						}
						else {
							vote_points += 0;
						}

					}

				}

			}

			mtx.lock();
			if (vote_points > score) {
				score = vote_points;
				image_vx = vy;
				image_vy = vx;

			}
			mtx.unlock();

		}

	}

}
void greatGaborKernels() {


	 double  sigma = 2.1;
	 int metr = 0;
	 for (int i = 0; i < 180; i = i + 5) {
	 	gabor_kernels_r.resize(gabor_kernels_r.size() + 1);
	 	gabor_kernels_i.resize(gabor_kernels_i.size() + 1);
	 	for (int j = 0; j < 5; j++) {
	 		int kernel_size = 21;
	 		cv::Mat dest;
	 		double th = i, ps = 0;
	 		cv::Mat kernelR = mgetGaborKernel(cv::Size(kernel_size, kernel_size), 2.2, (i * M_PI) / 180.0, (sigma * pow(2, j)) * M_PI / 180, 0.5, ps);
	 		cv::Mat kernelI = mgetGaborKernel(cv::Size(kernel_size, kernel_size), 2.2, (i * M_PI) / 180.0, (sigma * pow(2, j)) * M_PI / 180, 0.5, (ps * M_PI) / 180);
	 		gabor_kernels_r[metr].push_back(kernelR);
	 		gabor_kernels_i[metr].push_back(kernelI);
	 	}
	 	metr++;

	 }



}

void strofi(cv::Mat arxiki_eikona, cv::Mat keep_max_gwnia_xwris_maska, int xan_epafi, cv::Point& simeio) {

	//cv::Mat eikona = keep_max_gwnia_xwris_maska.clone();
	//keep_max_gwnia_xwris_maska = cv::Mat(keep_max_gwnia_xwris_maska, cv::Rect(0, 0, keep_max_gwnia_xwris_maska.cols, xan_epafi + 20)).clone();
	//C2DPoint  vanishing=C2DPoint(0,0); int analisi = 0; int str = 80;
	//std::vector<vpPair>  vp_pairs;
	//vanishing_point(keep_max_gwnia_xwris_maska, vanishing, vp_pairs);
	//cv::circle(arxiki_eikona, cv::Point(vanishing.y, vanishing.x), 4, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), cv::FILLED, 8);
	//
	//simeio = cv::Point(vanishing.y, vanishing.x);



}

std::vector<std::vector<cv::Mat>> real_m_dest(36, std::vector<cv::Mat>(5));
std::vector<std::vector<cv::Mat>> imag_m_dest(36, std::vector<cv::Mat>(5));
std::vector<cv::Mat> square_norm_r(36);
std::vector<cv::Mat> square_norm_i(36);

void aplay_filer1( cv::Mat  gabor_kernels , cv::Mat  eikona,int r_or_i, int i,int j)
{
	 cv::Mat  result1;
	 cv::Mat  result2;
	 cv::Mat  result3;
	 cv::Mat  result4;
	 cv::Mat  result5;

	 cv::filter2D( eikona, result1, CV_64F, gabor_kernels_r[i][0]);
	 cv::filter2D( eikona, result2, CV_64F, gabor_kernels_r[i][1]);
	 cv::filter2D( eikona, result3, CV_64F, gabor_kernels_r[i][2]);
	 cv::filter2D( eikona, result4, CV_64F, gabor_kernels_r[i][3]);
	 cv::filter2D( eikona, result5, CV_64F, gabor_kernels_r[i][4]);


	 result1.copyTo(real_m_dest[i][0]);
	 result2.copyTo(real_m_dest[i][1]);
	 result3.copyTo(real_m_dest[i][2]);
	 result4.copyTo(real_m_dest[i][3]);
	 result5.copyTo(real_m_dest[i][4]);


	 cv::filter2D( eikona, result1, CV_64F, gabor_kernels_i[i][0]);
	 cv::filter2D( eikona, result2, CV_64F, gabor_kernels_i[i][1]);
	 cv::filter2D( eikona, result3, CV_64F, gabor_kernels_i[i][2]);
	 cv::filter2D( eikona, result4, CV_64F, gabor_kernels_i[i][3]);
	 cv::filter2D( eikona, result5, CV_64F, gabor_kernels_i[i][4]);

	 result1.copyTo(imag_m_dest[i][0]);
	 result2.copyTo(imag_m_dest[i][1]);
	 result3.copyTo(imag_m_dest[i][2]);
	 result4.copyTo(imag_m_dest[i][3]);
	 result5.copyTo(imag_m_dest[i][4]);

}
void calc_sqrt(int rows,int cols, int i ) {


	(cv::Mat(rows,  cols, CV_64F, double(0))).copyTo(square_norm_r[i]);
	(cv::Mat(rows,  cols, CV_64F, double(0))).copyTo(square_norm_i[i]);

	for (int k = 0; k < rows; k++) {

		for (int l = 0; l < cols; l++) {

			for (int j = 0; j < 5; j++) {

				square_norm_r[i].at<double>(k, l) += pow(real_m_dest[i][j].at<double>(k, l), 2);
				square_norm_i[i].at<double>(k, l) += pow(imag_m_dest[i][j].at<double>(k, l), 2);

			}
			square_norm_r[i].at<double>(k, l) = square_norm_r[i].at<double>(k, l) / 5.;
			square_norm_i[i].at<double>(k, l) = square_norm_i[i].at<double>(k, l) / 5.;

		}

	}
}
cv::Mat keep_max_gwnia;
//typedef std::pair<float, float> Gabor_pair;


void keep_max_gwnia_t(int row,int cols , cv::Mat result_eik_patwma) {

	float max_apokrisi = -228360;
	float gwnia;
	int metr_gwn = 0;
	for (int l = 0; l <  cols; l++) {
		metr_gwn = 0;
		for (int i = 0; i < 36; i++) {

			float temp_apokrisi = pow(square_norm_r[i].at<double>(row, l), 2) + pow(square_norm_r[i].at<double>(row, l), 2);

			if (temp_apokrisi > max_apokrisi) {

				max_apokrisi = temp_apokrisi;
				gwnia = metr_gwn;

			}

			metr_gwn += 5;

		}

		if (result_eik_patwma.at<cv::Vec3b>(row, l) != cv::Vec3b(0, 0, 255)) {
			keep_max_gwnia.at<double>(row, l) = 228360;
		}
		else {
			keep_max_gwnia.at<double>(row, l) = gwnia;
		}
		max_apokrisi = -228360;

	}

}

void Image_pr::vanishing_point() {

	int bima = image_rows * 0.3 / 8;
	int k1 = 0; int k2 = bima;
	image_th[0] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[1] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[2] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[3] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[4] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[5] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
    image_th[6] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2); k1 = k2; k2 = k2 + bima;
	image_th[7] = std::thread(&Image_pr::find_votes_thead_l, this, k1,k2);


	for (int ii = 0; ii < 8; ++ii) { image_th[ii].join();  }
}
void  Image_pr::Garbor_filter(  ) {

	cv::Mat in ;
	cv::Mat hsv_imag;

	cv::cvtColor(image, hsv_imag, cv::COLOR_BGR2HSV);

	std::vector<cv::Mat> hsv_planes;
	cv::split(hsv_imag, hsv_planes);

	hsv_planes[2].copyTo(in);

	cv::Mat src_f;
	in.convertTo(src_f, CV_64F);

	int m_t = 0;
	int thread_nump  = 0  ;
///////////////////////////aply filter/////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < 36; i++) {
		if (thread_nump == 8) {
			for (int i = 0; i < thread_nump; ++i) {
				image_th[i].join();
			}
			thread_nump = 0;
		}
		image_th[thread_nump] = std::thread(aplay_filer1, gabor_kernels_r[i][0],  src_f, 0, i, 0);
		thread_nump++;

	}

	for (int i = 0; i < thread_nump; ++i) {

		image_th[i].join();

	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////find_sqrt_rooot of five scaling
	thread_nump = 0;
	for (int i = 0; i < 36; i++) {
		if (thread_nump == 8) {
			for (int i = 0; i < thread_nump; ++i) {
				image_th[i].join();
			}
			thread_nump = 0;
		}
		image_th[thread_nump] = std::thread(calc_sqrt, in.rows, in.cols, i);
		thread_nump++;

	}

	for (int i = 0; i < thread_nump; ++i) {
		image_th[i].join();
	}
	cv::Mat keep_max;

	keep_max = cv::Mat(in.rows, in.cols, CV_64F);
	keep_max_gwnia = cv::Mat(in.rows, in.cols, CV_64F);

	int metr;

	thread_nump = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int k = 0; k < in.rows; k++) {

		metr = 0;
		if (thread_nump == 8) {
			for (int i = 0; i < thread_nump; ++i) {
				image_th[i].join();
			}
			thread_nump = 0;
		}
		image_th[thread_nump] = std::thread(keep_max_gwnia_t, k, in.cols, maska_edafous);
		thread_nump++;
	}

	for (int i = 0; i < thread_nump; ++i) {
		image_th[i].join();
	}

	image_texture = keep_max_gwnia.clone();

    vanishing_point();

	return;

}

  Image_pr::Image_pr(cv::Mat image_g_f,  cv::Mat imageGround)
 {
 	     image = cv::Mat(image_g_f, cv::Rect(image_g_f.cols / 2 - image_g_f.cols / 4, 100, image_g_f.cols / 2, image_g_f.rows - 100)).clone();
		 maska_edafous = cv::Mat(imageGround, cv::Rect(imageGround.cols / 2 - imageGround.cols / 4, 100, imageGround.cols / 2, imageGround.rows - 100)).clone();
		 (*this).image_cols = image.cols;
		 (*this).image_rows = image.rows;
		 image_radius = 0.3 * sqrt(pow(image_rows, 2) + pow(image_cols, 2));

 }
