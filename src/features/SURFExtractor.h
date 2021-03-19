//
// Created by cv-bhlab on 3/19/21.
//

#ifndef HYSLAM_SURFEXTRACTOR_H
#define HYSLAM_SURFEXTRACTOR_H

#include <FeatureFinder.h>
#include <FeatureExtractor.h>
#include <DescriptorDistance.h>
#include <opencv/cv.h>

#include <memory>

namespace HYSLAM {

class SURFExtractor {
public:
    SURFExtractor(std::unique_ptr <FeatureFinder> feature_finder_, std::shared_ptr <DescriptorDistance> dist_func_,
                  ORBextractorSettings settings_);

    void operator()( cv::InputArray image, cv::InputArray mask,
                     std::vector<cv::KeyPoint>& keypoints, std::vector<FeatureDescriptor> &descriptors);


private:
    std::unique_ptr<FeatureFinder> feature_finder;
    std::shared_ptr<DescriptorDistance> dist_func;
    ORBextractorSettings settings;
};
}

#endif //HYSLAM_SURFEXTRACTOR_H
