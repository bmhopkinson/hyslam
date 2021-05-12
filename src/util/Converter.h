
#ifndef CONVERTER_H
#define CONVERTER_H

/*
 * class holding static member functions used to convert between data structures
 * most should be self explantory but are documented below
 * be careful with quaternion vector order conventions ( [w, x, y, z] vs. [x, y, z, w])
 */

#include <FeatureDescriptor.h>

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include <Eigen/Geometry>
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace HYSLAM
{
typedef Eigen::Transform<double ,3,Eigen::Isometry,Eigen::ColMajor> Isometry3;

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const std::vector<FeatureDescriptor> &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);  //converts from cv::Mat transform matrix to g2o::SE3Quat, translation + rotation in quaternion form.
   // static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
    static Isometry3 cvMatToIso3(cv::Mat m); //converts cv::Mat transform into Eigen Isometry3
    static cv::Mat   Iso3tocvMat(Isometry3 Teig);//converts Eigen Isometry3  into  cv::Mat transform

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3); //converts from g2o::SE3Quat (translation + rotation in quaternion) to  cv::Mat transform matrix (SE3)
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3); // converts from g2o::Sim3 (similarity) to cv::Mat form.
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);  //converts 4 x 4 eigen matrix to cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix3d &m); //converts 3 x 3 eigen matrix to cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);  //converts 3 row x 1 col Eigen to  3 row x 1 col cv::Mat
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);  //construct cv::Mat matrix representing SE3 transform from Eigen components: R rotation matrix, t translation vector

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);  //convert from cv::Mat 3 rows by 1 col to Eigen equivalent
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint); // convert from cv::Point3f to Eigen 3 x 1 matrix.
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3); //convert from 3 x 3 cv::Mat to Eighen equivalent.

    static std::vector<float> toQuaternion(const cv::Mat &M);  //convert from cv::Mat rotation matrix (3x3) to quaternion in x, y, z, w order (DIFFERENT BELOW!!!)
    static Eigen::Quaterniond toQuatEigen(const std::vector<double> q); //convert from raw quaternion in w, x, y, z order to Eigen form
    static std::vector<double> toQuatStdvec(Eigen::Quaterniond q_eig); //convert from  Eigen quaternion to raw quaternion in w, x, y, z order
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
