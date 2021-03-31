//
// Created by cv-bhlab on 3/19/21.
//

#include "SURFExtractor.h"
#include <algorithm>
#include <iostream>

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
    std::vector<cv::KeyPoint> keypoints_raw;
    feature_finder->detect(image, keypoints_raw); //detect features

    //filter keypoints by level and response

    std::vector< std::vector<cv::KeyPoint> > keypoints_bylevel;
    for(int i = 0 ; i < settings.nLevels; i++){
        keypoints_bylevel.push_back(std::vector<cv::KeyPoint>());
    }

    //sort by level
    for(auto it = keypoints_raw.cbegin(); it != keypoints_raw.cend(); ++it){
        cv::KeyPoint kpt = *it;
        int level = kpt.octave;
        keypoints_bylevel[level].push_back(kpt);
    }

    //sort by response - and truncate
    int target_kpts_per_level = settings.nFeatures/settings.nLevels;
    int n_deferred = 0;
    for(auto it = keypoints_bylevel.rbegin(); it != keypoints_bylevel.rend(); ++it){ //reverse iterate b/c higher octaves tend to have fewer features
        std::sort(it->begin(), it->end(), [](cv::KeyPoint &a, cv::KeyPoint &b){return a.response > b.response; });  //sorts in descending order
        int n_target = target_kpts_per_level + n_deferred;
        if(it->size() > n_target){
            it->resize(n_target);
            n_deferred = 0;
        } else {
            n_deferred  = n_target - it->size();
        }
    }

/*
    for(auto it = keypoints_bylevel.begin(); it != keypoints_bylevel.end(); ++it) {
        std::vector<cv::KeyPoint> kpts_sorted = *it;
       std::cout << "size: " << it->size() << std::endl;
        for(auto it2 = kpts_sorted.begin(); it2 != kpts_sorted.end(); ++it2){
            std::cout << "level: " << it2->octave << ", response: " << it2->response << std::endl;
        }
    }
*/

 //   if(keypoints.size() > settings.nFeatures) {
  //      keypoints.resize(settings.nFeatures); //keep only the nFeatures best features
  //  }
    keypoints.reserve(settings.nFeatures);
    for(auto it = keypoints_bylevel.begin(); it != keypoints_bylevel.end(); ++it) {
        keypoints.insert(keypoints.end(), it->begin(), it->end());
    }

    cv::Mat descriptors_raw;
    feature_finder->compute(image, keypoints, descriptors_raw); //compute descriptors


    //reformat descriptors
    int nkeypoints = keypoints.size();
    descriptors.reserve(nkeypoints);
    for(int j = 0; j < nkeypoints ; ++j){
        descriptors.push_back(FeatureDescriptor(descriptors_raw.row(j), dist_func )  );
    }

}

}
