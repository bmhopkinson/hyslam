
#include <ORBUtil.h>

namespace HYSLAM{

void ORBUtil::extractORB(FeatureExtractor* extractor, const cv::Mat &img, std::vector<cv::KeyPoint> &keys, std::vector<FeatureDescriptor> &descriptors){
    (*extractor)(img, cv::Mat(), keys, descriptors);
}

}
