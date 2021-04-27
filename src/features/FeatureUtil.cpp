
#include <FeatureUtil.h>

namespace HYSLAM{

void FeatureUtil::extractFeatures(FeatureExtractor* extractor, const cv::Mat &img, std::vector<cv::KeyPoint> &keys, std::vector<FeatureDescriptor> &desc){
    (*extractor)(img, cv::Mat(), keys, desc);
}

}
