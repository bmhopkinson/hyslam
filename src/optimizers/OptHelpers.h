#ifndef OPTHELPERS_H_
#define OPTHELPERS_H_

/*
 * helper functions for optimization:
 * ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C) - computes centroid of points in P (each column is a point), and
 *   returns centroid as C, and points relative to centroid as Pr.
 *
 * cv::Mat  ComputeSim3_Horn(cv::Mat &P1, cv::Mat &P2, bool FixScale, cv::Mat &R) - uses Horn 1987 method to determine a similarity transform that maps
 * from P2 from to P1 frame. P2 and P1 represent corresponding sets of points in different reference frames (one point per column).
 * e.g. apply transform to points in P2 will get them in reference from of P1. the similarity transform
 * is the return value of the function. the rotation component of the transform is passed back as R.
 *
 * PoseAlignmentTransform(std::vector<cv::Mat> &T1, std::vector<cv::Mat> &T2) - uses horn method to compute a transform aligning
 * poses represented by T2 with poses represented by T1. does this be creating a set of points representing each pose where one point
 * represents the pose origin and 3 additional points are unit vectors along x,y,z axis of pose (in reference frame T2 and T1 respectively).
 *
 * cv::Mat PosePoints(std::vector<cv::Mat> &Ts) - takes pose represented by T and returns a set of points representing the pose
 * where where one point represents the pose origin and 3 additional points are unit vectors along x,y,z axis of pose
 *
 * Eigen::Matrix<double, 3,3> Rotate_GpsError(std::vector<double> gpserr, cv::Mat &R) - rotates error of GPS measurements from gps frame of reference (gpserr)
 * into another frame as specified by rotation matrix R.
 *
 */

#include<vector>
#include<opencv2/core/core.hpp>
#include<Eigen/Core>

namespace HYSLAM
{


void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
cv::Mat ComputeSim3_Horn(cv::Mat &P1, cv::Mat &P2, bool FixScale, cv::Mat &R);
cv::Mat PoseAlignmentTransform(std::vector<cv::Mat> &T1, std::vector<cv::Mat> &T2);
cv::Mat PosePoints(std::vector<cv::Mat> &Ts);
Eigen::Matrix<double, 3,3> Rotate_GpsError(std::vector<double> gpserr, cv::Mat &R);

} //end ORB_SLAM2 namespace

#endif
