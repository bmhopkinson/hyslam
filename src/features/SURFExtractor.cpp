//
// Created by cv-bhlab on 3/19/21.
//

#include "SURFExtractor.h"

namespace HYSLAM{
SURFExtractor::SURFExtractor(std::unique_ptr <FeatureFinder> feature_finder_, std::shared_ptr <DescriptorDistance> dist_func_,
                             FeatureExtractorSettings settings_): dist_func(dist_func_), settings(settings_)
{
    feature_finder = std::move(feature_finder_);
}


void SURFExtractor::operator()( cv::InputArray _image, cv::InputArray mask,
                     std::vector<cv::KeyPoint>& keypoints, std::vector<FeatureDescriptor> &descriptors){
    if(_image.empty())
        return;

    cv::Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    cv::Mat descriptors_raw;
    feature_finder->detect(image, keypoints); //detect features
    feature_finder->compute(image, keypoints, descriptors_raw); //compute descriptors


    //reformat descriptors
    int nkeypoints = keypoints.size();
    descriptors.reserve(nkeypoints);
    for(int j = 0; j < nkeypoints ; ++j){
        descriptors.push_back(FeatureDescriptor(descriptors_raw.row(j), dist_func )  );
    }

}

}
