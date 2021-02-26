#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/opencv.hpp>
#include <string>

namespace HYSLAM{
class Camera
{
public:
    int loadData(const cv::FileNode &data_node);
    bool Project (cv::Mat Pc, cv::Mat &uv) const;
    cv::Mat Unproject (float u, float v, float z) const;


    std::string camName;
    int sensor; // sensor type; 0 mono, 1 stereo, 2 RGBD
    int RGB; // 0: BGR, 1: RGB.
    cv::Mat K; //calibration matrix
    cv::Mat distCoef; // distortion coefficients
    cv::Mat Tcam; //camera transform (body center -> camera center)
    cv::Mat Tcam_opt; //optimized camera transform (body center -> camera center)
    float mbf; //stereobase line
    float thDepth; //depth threshold
    float fps;
    float scale; //scale of image relative to original
    float mnMinX = 0.0;
    float mnMaxX;
    float mnMinY = 0.0;
    float mnMaxY;

    float fx() const {return K.at<float>(0,0);}
    float fy() const {return K.at<float>(1,1);}
    float cx() const {return K.at<float>(0,2);}
    float cy() const {return K.at<float>(1,2);}
    float mb() const {return mbf/K.at<float>(0,0);}

    
private:

};

}//close namespace

#endif
