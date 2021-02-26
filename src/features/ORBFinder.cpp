//
// Created by cv-bhlab on 2/26/21.
//

#include "ORBFinder.h"
#include <opencv2/features2d.hpp>
#include <cmath>


namespace HYSLAM {

ORBFinder::ORBFinder(double threshold_, bool non_max_suppression_) : non_max_suppression(non_max_suppression_)
{
    setThreshold(threshold_);
}

void ORBFinder::setThreshold(double threshold_){
    threshold = static_cast<int>(std::round(threshold));
}

double ORBFinder::getThreshold(){
    return static_cast<double>(threshold);
}

void ORBFinder::detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints){
    cv::FAST(image, keypoints, threshold, non_max_suppression);
}

}
