#ifndef GENUTILS_H_
#define GENUTILS_H_

/*
 * class containing general utility static member functions
 * fairly random assortment
 *
 * Epipole(KeyFrame* pKF1, KeyFrame* pKF2, float& ex, float& ey)- computes epipole of KF1 camera center in KF2
 *
 * ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled) - scales mVel (an incremental SE3 tranform rather than an actual velocity)
 *   which occurred over time vel_dt to the incremental tranform (mVel_scaled) that would have occurred in the motion had occured over time target_dt.
 *   assumes constant velocity over time increments. can handle forward or backward in time.
 *
 * cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2) - calculate (and return) Fundamental Matrix relating Keyframes pKF1 and pKF2
 *
 * cv::Mat SkewSymmetricMatrix(const cv::Mat &v); - generate (and return) skew symmetric matrix from 3 element vector (v)
 *
 * bool PointHasPositiveDepth(cv::Mat ProjectionMatrix, cv::Mat point) - returns true if point (3 vector or homogeneous 4 vector)
 *    is in front (positive depth) of the (linear) camera represented by ProjectionMatrix. Point must be in camera coordinates, not world
 *
 * mkdirRecursive(const char* pathname, mode_t mode) - creates the directory or directories (unix specific) specified in pathname with permissions
 *    determined by mode (mode_t declared in <sys/stat.h>). for full access use ACCESSPERMS
 *
 */

#include <KeyFrame.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

namespace HYSLAM{

struct GenUtils{
    static int Epipole(KeyFrame* pKF1, KeyFrame* pKF2, float& ex, float& ey);  //computes epipole of KF1 camera center in KF2
    static void ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled);
    static cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
    static cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
    static bool PointHasPositiveDepth(cv::Mat ProjectionMatrix, cv::Mat point);
    static int mkdirRecursive(const char* pathname, mode_t mode);
};

}

#endif
