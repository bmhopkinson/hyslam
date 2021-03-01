#ifndef ORBUTIL_H_
#define ORBUTIL_H_

#include <FeatureExtractor.h>

namespace HYSLAM{


class ORBUtil{
    public:
    static void extractORB(FeatureExtractor* extractor, const cv::Mat &img, std::vector<cv::KeyPoint> &keys, cv::Mat &desc);
};

}

#endif
