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

    virtual void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints) = 0;

};

}
#endif //HYSLAM_FEATUREFINDER_H
