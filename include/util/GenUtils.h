#ifndef GENUTILS_H_
#define GENUTILS_H_

#include <KeyFrame.h>
#include <opencv2/opencv.hpp>

namespace HYSLAM{

struct GenUtils{
    static int Epipole(KeyFrame* pKF1, KeyFrame* pKF2, float& ex, float& ey);  //computes epipole of KF1 camera center in KF2
    static void ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled);
    static cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
    static cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
    static bool PointHasPositiveDepth(cv::Mat ProjectionMatrix, cv::Mat point);
};

}

#endif
