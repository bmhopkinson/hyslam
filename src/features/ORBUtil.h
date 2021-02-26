#ifndef ORBUTIL_H_
#define ORBUTIL_H_

#include <ORBextractor.h>

namespace HYSLAM{


class ORBUtil{
    public:
    static void extractORB(ORBextractor* extractor, const cv::Mat &img,  std::vector<cv::KeyPoint> &keys, cv::Mat &desc);
};

}

#endif
