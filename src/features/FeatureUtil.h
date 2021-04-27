#ifndef FEATUREUTIL_H_
#define FEATUREUTIL_H_

/*
 * Utility Functions for feature classes
 * extractFeatures(FeatureExtractor* extractor, const cv::Mat &img, std::vector<cv::KeyPoint> &keys, std::vector<FeatureDescriptor> &desc)
 *      wrapper to enable multithreading of feature extraction. the feature extractor finds features in img and returns feature keypoints in key and descriptors in desc
 *
 */

#include <FeatureExtractor.h>

namespace HYSLAM{


class FeatureUtil{
    public:
    static void extractFeatures(FeatureExtractor* extractor, const cv::Mat &img, std::vector<cv::KeyPoint> &keys, std::vector<FeatureDescriptor> &desc);
};

}

#endif
