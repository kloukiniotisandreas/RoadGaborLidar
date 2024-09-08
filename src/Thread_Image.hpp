#pragma once
#include <iostream>
#include <thread>
#include <future>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include <mutex>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat  mgetGaborKernel(cv::Size ksize, double sigma, double theta, double lambd, double gamma, double psi);
void aplay_filer1(cv::Mat  gabor_kernels, cv::Mat eikona, int r_or_i, int i, int j);
void calc_sqrt(int rows, int cols, int i);
void great_gabor_kernels();
void keep_max_gwnia_t(int row, int cols, cv::Mat result_eik_patwma);
void strofi(cv::Mat arxiki_eikona, cv::Mat keep_max_gwnia_xwris_maska, int xan_epafi, cv::Point& simeio);

class Image_pr {
public:

	std::thread image_th[8];
	int image_cols;
	int image_rows;
	int image_radius;
	float image_vx, image_vy;
	float score = std::numeric_limits<float>::min();
	cv::Mat image_texture;
	cv::Mat maska_edafous;
	cv::Mat image;
	cv::Mat final_image;

	std::mutex mtx;
	Image_pr(cv::Mat image_g_f, cv::Mat result_eik_patwma);
	void Garbor_filter();
	void vanishing_point();
	void find_votes_thead_l(int vy1, int vy2);

};
