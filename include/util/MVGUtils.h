#ifndef MVGUTILS_H_
#define MVGUTILS_H_

#include <KeyFrame.h>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2{

struct MVGUtils{
    static int Epipole(KeyFrame* pKF1, KeyFrame* pKF2, float& ex, float& ey);  //computes epipole of KF1 camera center in KF2

};

}

#endif
