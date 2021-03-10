//
// Created by cv-bhlab on 2/26/21.
//

#ifndef HYSLAM_FEATUREFINDER_H
#define HYSLAM_FEATUREFINDER_H

#include <opencv2/core/core.hpp>
#include <vector>

namespace HYSLAM {

class FeatureFinder {
public:
    virtual void setThreshold(double threshold_) = 0;

    virtual double getThreshold() = 0;

    virtual void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints) = 0; //detect features
    virtual void compute(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat& descriptors) = 0; //compute descriptors

    virtual int descriptor_cols() = 0; //number of columns needed for a single descriptor
    virtual int descriptor_mat_type()= 0; //CV MAT types:
    // CV_8U : 0
    // CV_8S : 1
    // CV_16U : 2
    // CV_16S : 3
    // CV_32S : 4
    // CV_32F : 5
    // CV_64F : 6
    // CV_16F : 7
};

}
#endif //HYSLAM_FEATUREFINDER_H
