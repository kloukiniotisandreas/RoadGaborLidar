#pragma once

#include <chrono>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/date_time.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32


void findknn(std::vector<float> point, std::vector<int> &place_knn, int place_point);

typedef struct {

    long timestamp;

    // lat:   latitude of the oxts-unit (deg)
    double lat;
    // lon:   longitude of the oxts-unit (deg)
    double lon;
    // alt:   altitude of the oxts-unit (m)
    double alt;
    // roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    double roll;
    // pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    double pitch;
    // yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    double yaw;


    // vn:    velocity towards north (m/s)
    double vn;
    // ve:    velocity towards east (m/s)
    double ve;
    // vf:    forward velocity, i.e. parallel to earth-surface (m/s)
    double vf;
    // vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
    double vl;
    // vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
    double vu;


    // ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
    double ax;
    // ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
    double ay;
    // az:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
    double az;
    // af:    forward acceleration (m/s^2)
    double af;
    // al:    leftward acceleration (m/s^2)
    double al;
    // au:    upward acceleration (m/s^2)
    double au;
    // wx:    angular rate around x (rad/s)
    double wx;
    // wy:    angular rate around y (rad/s)
    double wy;
    // wz:    angular rate around z (rad/s)
    double wz;
    // wf:    angular rate around forward axis (rad/s)
    double wf;
    // wl:    angular rate around leftward axis (rad/s)
    double wl;
    // wu:    angular rate around upward axis (rad/s)
    double wu;


    // pos_accuracy:  velocity accuracy (north/east in m)
    double pos_accuracy;
    // vel_accuracy:  velocity accuracy (north/east in m/s)
    double vel_accuracy;
    // navstat:       navigation status (see navstat_to_string)
    int navstat;
    // numsats:       number of satellites tracked by primary GPS receiver
    int numsats;
    // posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
    int posmode;
    // velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
    int velmode;
    // orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
    int orimode;

} gpsimu_t;

typedef struct {

    long timestamp;
    long timestamp_start;
    long timestamp_end;

    int num_points;

    std::vector<std::array<float,4>> points;


} lidar_t;

typedef struct {

    long timestamp;

    bool is_color;

    int width;
    int height;

    cv::Mat image_left;
    cv::Mat image_right;

} stereo_t;

class Kitti_dataset{

    public:
        std::string datasetPath;
        std::vector<std::string> lidars;
        std::vector<std::string> images;
        int FrameNumber;

    Kitti_dataset(int FrameNumber,std::string path){
        (*this).datasetPath = path;
        (*this).FrameNumber = FrameNumber;

        (*this).load_pathFiles(path,"velodyne_points/data/", (*this).FrameNumber, (*this).lidars);
        (*this).load_pathFiles(path,"image_02/data/", (*this).FrameNumber, (*this).images);

    }


    void load_pathFiles(std::string path,std::string subFolderPath,int FrameNumber, std::vector<std::string> &pathFiles);

};

 void calculateDominantDeriction(Kitti_dataset dataset);
