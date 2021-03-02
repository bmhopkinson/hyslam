//
// Created by cv-bhlab on 3/2/21.
//

#ifndef HYSLAM_SURFFINDER_H
#define HYSLAM_SURFFINDER_H

#include <FeatureFinder.h>
#include "opencv2/xfeatures2d.hpp"

namespace HYSLAM {
class SURFFinder : public FeatureFinder {
public:
    SURFFinder();
    SURFFinder(double threshold_);
    void setThreshold(double threshold_){threshold = threshold_;};
    double getThreshold();
    void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints); //detect features
    void compute(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) ; //compute descriptors
private:
    double threshold = 20;
    int nOctaves =1;
    int nOctaveLayers = 1;
    cv::Ptr<cv::xfeatures2d::SURF> detector;
};

} //end namespace
#endif //HYSLAM_SURFFINDER_H
