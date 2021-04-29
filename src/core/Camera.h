#ifndef CAMERA_H_
#define CAMERA_H_

/*
 * implements a linear (pinhole) monocular or stereo camera model (despite storing distortion coefficient - these are not used)
 * loadData() loads camera data from stored file
 * Project(cv::Mat Pc, cv::Mat &uv) projects the 3D point in camera coordinates Pc to 2D camera location uv (uv = row, col; if stereo also includes col in right image as 3rd element)
 *      returns true if uv is in image, false if not
 * UnProject(u, v, z) returns a 3D point on the ray from the camera center through image pixel uv at distance z along the principal axis - i.e. unprojects uv to 3D point in camera coordinates w/ z-coordinate z
 *
 */

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
    float mnMinX = 0.0; //minimum image column
    float mnMaxX;   //maximum image column
    float mnMinY = 0.0; //minimum image row;
    float mnMaxY;   //maximum image row

    float fx() const {return K.at<float>(0,0);}
    float fy() const {return K.at<float>(1,1);}
    float cx() const {return K.at<float>(0,2);}
    float cy() const {return K.at<float>(1,2);}
    float mb() const {return mbf/K.at<float>(0,0);}

    
private:

};

}//close namespace

#endif
