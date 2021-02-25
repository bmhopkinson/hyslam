#ifndef TRIANGULATOR_H_
#define TRIANGULATOR_H_

#include <opencv2/opencv.hpp>
//only does Direct linear triangulation and no state right now - imagine it will expand

namespace ORB_SLAM2{
class Triangulator{
public:
    bool DirectLinearTriangulation(cv::Mat uv1, cv::Mat uv2, cv::Mat P1, cv::Mat P2, cv::Mat &x3D);
};
}//end namespace

#endif