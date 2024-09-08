#include "SceneMesh3D.hpp"
#include "RoadDetection.hpp"
#include "Thread_Image.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

using namespace pcl;

typedef std::pair<int, double> myPair45;

std::vector<std::string>
Kitti_dataset::load_pathFiles(const std::string &path_lidar,
                              const std::string &subFolderPath,
                              const int &frameNumber) {

  std::vector<std::string> files;
  files.reserve(frameNumber);

  // Loop through all sub folders, assume they are sequential
  // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
  boost::filesystem::path p(path_lidar + subFolderPath);

  std::vector<boost::filesystem::path> v;
  std::copy(boost::filesystem::directory_iterator(p),
            boost::filesystem::directory_iterator(), std::back_inserter(v));

  // Sort, since directory iteration
  // Is not ordered on some file systems
  std::sort(v.begin(), v.end());
  // Append them
  for (std::vector<boost::filesystem::path>::const_iterator it(v.begin()),
       it_end(v.begin() + frameNumber);
       it != it_end; ++it) {
    files.push_back((*it).c_str());
  }
  return files;
}

void mult(std::vector<std::vector<float>> A, std::vector<std::vector<float>> B,
          std::vector<std::vector<float>> &C) {

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

void antistixeia_simeiwn_t(pcl::PointCloud<pcl::PointXYZ>::Ptr final, int prwto,
                           int deutero, std::vector<std::vector<float>> P,
                           std::vector<std::vector<float>> R,
                           std::vector<std::vector<float>> T_velo, int cols,
                           int rows) {

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

    if ((u >= cols || u <= 0 || v >= rows || v <= 0)) {
    } else {
      cv::circle(result_eik_patwma_02, cv::Point(u, v), 10,
                 cv::Scalar(0, 0, 255), cv::FILLED, 8);
    }
  }
}
void map_lidar_points_to_image_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr final,
                                     cv::Mat irt65,
                                     cv::Mat &result_eik_koukides) {

  result_eik_patwma_02 = irt65.clone();

  std::vector<std::vector<float>> P(3);
  P[0].push_back(7.215377e+02);
  P[0].push_back(0.000000e+00);
  P[0].push_back(6.095593e+02);
  P[0].push_back(4.485728e+01);
  P[1].push_back(0.000000e+00);
  P[1].push_back(7.215377e+02);
  P[1].push_back(1.728540e+02);
  P[1].push_back(2.163791e-01);
  P[2].push_back(0.000000e+00);
  P[2].push_back(0.000000e+00);
  P[2].push_back(1.000000e+00);
  P[2].push_back(2.745884e-03);

  std::vector<std::vector<float>> R(4);
  R[0].push_back(9.999239e-01);
  R[0].push_back(9.837760e-03);
  R[0].push_back(-7.445048e-03);
  R[0].push_back(0);
  R[1].push_back(-9.869795e-03);
  R[1].push_back(9.999421e-01);
  R[1].push_back(-4.278459e-03);
  R[1].push_back(0);
  R[2].push_back(7.402527e-03);
  R[2].push_back(4.351614e-03);
  R[2].push_back(9.999631e-01);
  R[2].push_back(0);
  R[3].push_back(0);
  R[3].push_back(0);
  R[3].push_back(0);
  R[3].push_back(1);

  std::vector<std::vector<float>> T_velo(4);
  T_velo[0].push_back(7.533745e-03);
  T_velo[0].push_back(-9.999714e-01);
  T_velo[0].push_back(-6.166020e-04);
  T_velo[0].push_back(-4.069766e-03);
  T_velo[1].push_back(1.480249e-02);
  T_velo[1].push_back(7.280733e-04);
  T_velo[1].push_back(-9.998902e-01);
  T_velo[1].push_back(-7.631618e-02);
  T_velo[2].push_back(9.998621e-01);
  T_velo[2].push_back(7.523790e-03);
  T_velo[2].push_back(1.480755e-02);
  T_velo[2].push_back(-2.717806e-01);
  T_velo[3].push_back(0);
  T_velo[3].push_back(0);
  T_velo[3].push_back(0);
  T_velo[3].push_back(1);

  cv::Mat img_rgb;
  cv::cvtColor(irt65, img_rgb, cv::COLOR_BGR2RGB);

  int bima = final->size() / 4;
  std::thread thread[4];

  int prwto = 0;
  int deutero = bima;
  thread[0] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R,
                          T_velo, irt65.cols, irt65.rows);
  prwto = deutero;
  deutero = deutero + bima;
  thread[1] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R,
                          T_velo, irt65.cols, irt65.rows);
  prwto = deutero;
  deutero = deutero + bima;
  thread[2] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R,
                          T_velo, irt65.cols, irt65.rows);
  prwto = deutero;
  deutero = final->size();
  thread[3] = std::thread(antistixeia_simeiwn_t, final, prwto, deutero, P, R,
                          T_velo, irt65.cols, irt65.rows);

  for (int i = 0; i < 4; ++i) {
    thread[i].join();
  }

  return;
}
void load_lidar_points(std::string onoma_bin,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  std::vector<std::vector<float>> lidar_a;
  int num = 1000000;
  float *data = (float *)malloc(num * sizeof(float));

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
    px += 4;
    py += 4;
    pz += 4;
    pr += 4;
  }
  fclose(stream);

  for (int i = 0; i < lidar_a.size(); i++) {

    if (lidar_a[i][0] >= -1 && lidar_a[i][2] < -1.2) {
      cloud->points.push_back(
          pcl::PointXYZ(lidar_a[i][0], lidar_a[i][1], lidar_a[i][2]));
    }
  }
}

void calculateDominantDeriction(Kitti_dataset dataset) {

  greatGaborKernels();

  for (int i = 0; i < dataset.images.size(); i++) {

    cv::Mat imageLidarProgected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "[vp_detection]: lidar loading \n" << std::endl;
    load_lidar_points(dataset.lidars[i], cloud);
    cv::Mat arxiki_eikona = cv::imread(dataset.images[i], cv::IMREAD_COLOR);
    cv::Mat irt = cv::imread(dataset.images[i], cv::IMREAD_COLOR);

    pcl_segmentation(cloud, final);
    std::cout << "[vp_detection]: Ground points \n" << std::endl;
    map_lidar_points_to_image_plane(final, arxiki_eikona, imageLidarProgected);

    Image_pr frame(irt, result_eik_patwma_02);

    frame.Garbor_filter();
    std::vector<std::string> result_name;
    boost::split(result_name, dataset.images[i], boost::is_any_of("\\/"));
    // recalculate_v_p(C2DPoint(frame.image_vx, frame.image_vy),
    // frame.image_texture,frame.image, result_name.back(), frame.final_image);
    cv::imshow("VP and dominantn boarder", frame.final_image);
  }

  return;
}

// Using OpenCV types to replace C2DPoint, C2DLine, and vec
std::vector<std::vector<float>> simeia_dromou;
std::vector<std::vector<float>> simeia_dromou_patwma;

float find_mena_var_colour(cv::Mat image_aa, cv::Point2f startp,
                           cv::Point2f endp, cv::Vec3f &mean1, cv::Vec3f &mean2,
                           cv::Mat orientation) {

  int bima = 15;
  cv::Mat perioxi2;
  cv::Mat black_imag(image_aa.rows, image_aa.cols, CV_8UC3,
                     cv::Scalar(0, 0, 0));

  // Define lines using startp and endp with some shift
  cv::Point2f shift_vec(bima, 0);
  cv::Point2f startp_shifted1 = startp + shift_vec;
  cv::Point2f endp_shifted1 = endp + shift_vec;
  cv::Point2f startp_shifted2 = startp - shift_vec;
  cv::Point2f endp_shifted2 = endp - shift_vec;

  float paxos = 2 * cv::norm(startp - startp_shifted1);

  // Draw shifted lines
  line(black_imag, startp_shifted1, endp_shifted1, cv::Scalar(0, 0, 255), paxos,
       8);
  line(black_imag, startp_shifted2, endp_shifted2, cv::Scalar(255, 0, 255),
       paxos, 8);

  mean1 = cv::Vec3f(0, 0, 0);
  mean2 = cv::Vec3f(0, 0, 0);
  cv::Vec3f var(0, 0, 0);
  int arithmos_pixel1 = 0;
  int arithmos_pixel2 = 0;

  // First pass to compute mean
  for (int k = 0; k < black_imag.cols; k++) {
    for (int l = 0; l < black_imag.rows; l++) {
      cv::Vec3b color2 = black_imag.at<cv::Vec3b>(l, k);
      if (color2 == cv::Vec3b(0, 0, 255) &&
          orientation.at<double>(l, k) != 228360) {
        cv::Vec3b color3 = image_aa.at<cv::Vec3b>(l, k);
        mean1 += cv::Vec3f(color3[0], color3[1], color3[2]);
        arithmos_pixel1++;
      } else if (color2 == cv::Vec3b(255, 0, 255) &&
                 orientation.at<double>(l, k) != 228360) {
        cv::Vec3b color4 = image_aa.at<cv::Vec3b>(l, k);
        mean2 += cv::Vec3f(color4[0], color4[1], color4[2]);
        arithmos_pixel2++;
      }
    }
  }

  if (arithmos_pixel1 > 0)
    mean1 /= arithmos_pixel1;
  if (arithmos_pixel2 > 0)
    mean2 /= arithmos_pixel2;

  arithmos_pixel1 = 0;
  arithmos_pixel2 = 0;

  cv::Vec3f var1(0, 0, 0);
  cv::Vec3f var2(0, 0, 0);

  // Second pass to compute variance
  for (int k = 0; k < black_imag.cols; k++) {
    for (int l = 0; l < black_imag.rows; l++) {
      cv::Vec3b color2 = black_imag.at<cv::Vec3b>(l, k);
      if (color2 == cv::Vec3b(0, 0, 255) &&
          orientation.at<double>(l, k) != 228360) {
        cv::Vec3b color3 = image_aa.at<cv::Vec3b>(l, k);
        var1 += cv::Vec3f(pow((color3[0] - mean1[0]), 2),
                          pow((color3[1] - mean1[1]), 2),
                          pow((color3[2] - mean1[2]), 2));
        arithmos_pixel1++;
      } else if (color2 == cv::Vec3b(255, 0, 255) &&
                 orientation.at<double>(l, k) != 228360) {
        cv::Vec3b color4 = image_aa.at<cv::Vec3b>(l, k);
        var2 += cv::Vec3f(pow((color4[0] - mean2[0]), 2),
                          pow((color4[1] - mean2[1]), 2),
                          pow((color4[2] - mean2[2]), 2));
        arithmos_pixel2++;
      }
    }
  }

  if (arithmos_pixel1 > 0)
    var1 /= arithmos_pixel1;
  if (arithmos_pixel2 > 0)
    var2 /= arithmos_pixel2;

  float diafx = abs(mean1[0] - mean2[0]) / sqrt(var1[0] + var2[0]);
  float diafy = abs(mean1[1] - mean2[1]) / sqrt(var1[1] + var2[1]);
  float diafz = abs(mean1[2] - mean2[2]) / sqrt(var1[2] + var2[2]);

  if (arithmos_pixel1 == 0 || arithmos_pixel2 == 0)
    return 0;
  if (diafx > diafy && diafx > diafz)
    return diafx;
  if (diafy > diafx && diafy > diafz)
    return diafy;
  if (diafz > diafx && diafz > diafy)
    return diafz;
}

float find_ocr_space(cv::Mat image_aa, cv::Point2f startp, cv::Mat orientation,
                     cv::Point2f endp) {
  typedef std::pair<int, double> myPair;
  struct compare2 {
    bool operator()(myPair &l, myPair &r) { return l.second > r.second; }
  } cmp1;

  std::vector<myPair> pair_xanei_epafi;
  float ratio_ocr = 0;
  float simeia_gramis = 0;

  cv::Mat hsv_imag;
  cv::cvtColor(image_aa, hsv_imag, cv::COLOR_BGR2HSV);

  cv::Mat in;
  cv::split(hsv_imag, in);

  cv::Point2f simeio_tomis = endp;
  cv::Vec3f mean_1(0, 0, 0), mean_2(0, 0, 0);

  float diafora_xr = find_mena_var_colour(image_aa, startp, simeio_tomis,
                                          mean_1, mean_2, orientation);

  cv::Vec3f mean(0, 0, 0);
  double mikos_g = cv::norm(endp - startp);
  double bima = 1 / mikos_g;
  bool simea = false;

  for (double i = 0; i < 1; i += bima) {
    int k = startp.y + i * (endp.y - startp.y);
    int l = startp.x + i * (endp.x - startp.x);

    if (!(k < 0 || l < 0 || l >= orientation.cols || k >= orientation.rows) &&
        orientation.at<double>(k, l) != 228360) {
      double gwnia = orientation.at<double>(k, l) * M_PI / 180;
      cv::Point2f temp_end(l + 15 * cos(gwnia), k - 15 * sin(gwnia));
      cv::Vec2f line_vec(endp.x - startp.x, endp.y - startp.y);
      cv::Vec2f temp_vec(temp_end.x - l, temp_end.y - k);

      double dot_product =
          (line_vec[0] * temp_vec[0] + line_vec[1] * temp_vec[1]) /
          (cv::norm(line_vec) * cv::norm(temp_vec));

      if (dot_product >= 0.92) {
        pair_xanei_epafi.push_back(myPair(k, 0));
        ratio_ocr += 1;
      }
    }
    simeia_gramis++;
  }

  if (simeia_gramis == 0)
    return 0;

  return ratio_ocr / simeia_gramis;
}

void recalculate_v_p(cv::Point v_p, const cv::Mat &texture_orientation_a,
                     const cv::Mat &image_g_f, const std::string &name,
                     cv::Mat &result) {

  // Initialize max and min percentage
  float max_per = -228360;
  float min_per = 228360;

  // Clone the texture orientation matrix
  cv::Mat texture_orientation = texture_orientation_a.clone();

  // Convert the input image to grayscale
  cv::Mat gray_image;
  cv::cvtColor(image_g_f, gray_image, cv::COLOR_BGR2GRAY);

  // Clone the original image for further processing
  cv::Mat image_clone = image_g_f.clone();

  // Get the coordinates of the input point
  int max_col =
      v_p.x; // Using v_p.x instead of v_p.y, assuming v_p is a (x,y) point
  int max_row = v_p.y;

  // Initialize a vector of rays represented as pairs of points
  std::vector<std::pair<cv::Point, cv::Point>> rays;
  for (int i = 15; i < 170; i += 5) {
    float angle_rad = i * M_PI / 180;
    float cos_angle = cos(angle_rad);
    float sin_angle = sin(angle_rad);

    int end_x =
        max_col - ((image_clone.rows - max_row) / sin_angle) * cos_angle;
    int end_y =
        max_row + ((image_clone.rows - max_row) / sin_angle) * sin_angle;

    rays.emplace_back(cv::Point(max_col, max_row), cv::Point(end_x, end_y));
  }

  // Structs for sorting based on scores
  typedef std::pair<int, double> MyPair;

  struct CompareByScore {
    bool operator()(const MyPair &l, const MyPair &r) const {
      return l.second > r.second;
    }
  };

  // Vectors to hold OCR and disconnection scores
  std::vector<MyPair> ocr;
  std::vector<MyPair> disconnection_points;

  // Initialize with starting point
  ocr.push_back(MyPair(0, 0));
  disconnection_points.push_back(MyPair(0, 0));

  // Process each ray to find OCR score and disconnection point
  for (size_t i = 1; i < rays.size() - 1; ++i) {
    ocr.push_back(MyPair(i, 0));
    disconnection_points.push_back(MyPair(i, 0));

    float ocr_ratio = find_ocr_space(image_clone, cv::Point(max_col, max_row),
                                     texture_orientation, rays[i].second);

    // Update max and min OCR ratios
    max_per = std::max(max_per, ocr_ratio);
    min_per = std::min(min_per, ocr_ratio);

    // Store the results
    ocr.back().second = ocr_ratio;
    disconnection_points.back().second =
        0; // Placeholder for xanei_epafi (disconnection)
  }

  // Finalize the vectors by adding the last ray
  ocr.push_back(MyPair(rays.size() - 1, 0));
  disconnection_points.push_back(MyPair(disconnection_points.size() - 1, 0));

  // Sort OCR based on score in descending order
  std::sort(ocr.begin(), ocr.end(), CompareByScore());

  // Retrieve the disconnection point based on the best OCR score
  int disconnect_point = disconnection_points[ocr[0].first].second;

  // Get the best scoring ray and its direction vector
  cv::Point best_ray_start = rays[ocr[0].first].first;
  cv::Point best_ray_end = rays[ocr[0].first].second;
  cv::Point2f best_ray_vector = best_ray_end - best_ray_start;

  // Calculate the angle of the best ray
  double line_angle = atan2(best_ray_vector.y, best_ray_vector.x);

  // Draw lines and arrowed lines on the image
  cv::arrowedLine(image_clone, cv::Point(0, disconnect_point),
                  cv::Point(image_clone.cols, disconnect_point),
                  cv::Scalar(0, 220, 0), 2, 8);
  cv::Point mid_point = best_ray_start + 0.5 * (best_ray_end - best_ray_start);
  cv::arrowedLine(image_clone, cv::Point(max_col, max_row), mid_point,
                  cv::Scalar(255, 255, 125), 2, 8);
  cv::arrowedLine(image_clone, cv::Point(max_col, max_row), mid_point,
                  cv::Scalar(255, 0, 125), 2, 8);

  // Draw the point at the center
  cv::circle(image_clone, cv::Point(max_col, max_row), 5, cv::Scalar(0, 255, 0),
             cv::FILLED, 8);

  // Output the resulting image
  result = image_clone.clone();
}
