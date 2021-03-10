//
// Created by cv-bhlab on 3/10/21.
//

#ifndef HYSLAM_DESCRIPTORDISTANCE_H
#define HYSLAM_DESCRIPTORDISTANCE_H


#include <opencv2/core/core.hpp>

namespace HYSLAM {

class DescriptorDistance {
public:
    virtual int distance(const cv::Mat &D1, const cv::Mat &D2) = 0;

};

class ORBDistance : public DescriptorDistance{
    int distance(const cv::Mat &D1, const cv::Mat &D2);
};

}

#endif //HYSLAM_DESCRIPTORDISTANCE_H
