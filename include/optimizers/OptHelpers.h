#ifndef OPTHELPERS_H_
#define OPTHELPERS_H_

#include<vector>
#include<opencv2/core/core.hpp>
#include<Eigen/Core>

namespace ORB_SLAM2
{


void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
cv::Mat ComputeSim3_Horn(cv::Mat &P1, cv::Mat &P2, bool FixScale, cv::Mat &R);
cv::Mat PoseAlignmentTransform(std::vector<cv::Mat> &T1, std::vector<cv::Mat> &T2);
cv::Mat PosePoints(std::vector<cv::Mat> &Ts);
Eigen::Matrix<double, 3,3> Rotate_GpsError(std::vector<double> gpserr, cv::Mat &R);

} //end ORB_SLAM2 namespace

#endif
