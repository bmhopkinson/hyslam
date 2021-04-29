//
// Created by cv-bhlab on 3/10/21.
//

#ifndef HYSLAM_DESCRIPTORDISTANCE_H
#define HYSLAM_DESCRIPTORDISTANCE_H

/*
 *  classes used to calculate distance between Feature Descriptor raw cv::Mats
 *  DescriptorDistance is the abstract base class and defines the interface for the only function:
 *  float distance(const cv::Mat &D1, const cv::Mat &D2) - returns float which accomodates both binary and continous descriptors
 *
 *  the concrete classes correspond to Features:
 *  ORBDistance - Hamming Distance
 *  SURFDistance - L1 Norm
 */

#include <opencv2/core/core.hpp>

namespace HYSLAM {

class DescriptorDistance {
public:
    virtual float distance(const cv::Mat &D1, const cv::Mat &D2) = 0;

};

class ORBDistance : public DescriptorDistance{
    float distance(const cv::Mat &D1, const cv::Mat &D2);
};

class SURFDistance : public DescriptorDistance{
    float distance(const cv::Mat &D1, const cv::Mat &D2);
};

}

#endif //HYSLAM_DESCRIPTORDISTANCE_H
