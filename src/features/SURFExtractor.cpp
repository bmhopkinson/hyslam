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

    int N_ROWS = settings.N_CELLS;
    int N_COLS = settings.N_CELLS;
    int col_interval = floor(static_cast<double>(image.cols)/static_cast<double>(N_COLS));  //taking floor will lead to minor truncation of image
    int row_interval = floor(static_cast<double>(image.rows)/static_cast<double>(N_ROWS));
    int target_keypoints_per_section = ceil(static_cast<double>(settings.nFeatures)/static_cast<double>(N_COLS*N_ROWS));

   // std::vector<cv::KeyPoint> keypoints_raw;
    keypoints.reserve(settings.nFeatures);
    for(int i  = 0; i < N_ROWS; i++){
        for(int j = 0; j < N_COLS; j++){
            int x_min = col_interval*j;
            int x_max = col_interval*(j+1);
            int y_min = row_interval*i;
            int y_max = row_interval*(i+1);

            std::vector<cv::KeyPoint> keypoints_patch;
            feature_finder->detect(image.rowRange(y_min, y_max).colRange(x_min, x_max), keypoints_patch); //detect features

            for(auto it = keypoints_patch.begin(); it != keypoints_patch.end(); ++it){
                cv::Point2f xy_global = it->pt;
                xy_global.x = xy_global.x + x_min;
                xy_global.y = xy_global.y + y_min;
                it->pt = xy_global;
            }

            //filter keypoints by level and response

            std::vector< std::vector<cv::KeyPoint> > keypoints_bylevel;
            for(int i = 0 ; i < settings.nLevels; i++){
                keypoints_bylevel.push_back(std::vector<cv::KeyPoint>());
            }

            //sort by level
            for(auto it = keypoints_patch.cbegin(); it != keypoints_patch.cend(); ++it){
                cv::KeyPoint kpt = *it;
                int level = kpt.octave;
                keypoints_bylevel[level].push_back(kpt);
            }

            //sort by response - and truncate
            int target_kpts_per_level = ceil(static_cast<double>(target_keypoints_per_section)/static_cast<double>(settings.nLevels));
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


            for(auto it = keypoints_bylevel.begin(); it != keypoints_bylevel.end(); ++it) {
                keypoints.insert(keypoints.end(), it->begin(), it->end());
            }
        }
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
