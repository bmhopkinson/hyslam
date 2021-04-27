//
// Created by cv-bhlab on 3/19/21.
//

#ifndef HYSLAM_SURFEXTRACTOR_H
#define HYSLAM_SURFEXTRACTOR_H

/*
 * implements FeatureExtractor for SURF Features
 *  SURFExtractor(std::unique_ptr<FeatureFinder> feature_finder_, std::shared_ptr<DescriptorDistance> dist_func_, FeatureExtractorSettings settings)
 *      constructor is used to obtain the lower-level feature_finder object (must be SURFFinder ), appropriate feature descriptor distance function and settings
 *  the extraction function ()overload finds number of keypoints requested in settings distributed spatially by breaking the image into N_CELLS subsections horizontally and vertically (NCELLS typicall 2 or 3)
 *  and distributed over size by attempting to find roughly equal numbers of keypoints in each octave (this is often difficult with largest scale features)
 */

#include <FeatureExtractor.h>
#include <FeatureFinder.h>
#include <FeatureDescriptor.h>
#include <DescriptorDistance.h>
#include <opencv/cv.h>

#include <memory>

namespace HYSLAM {

class SURFExtractor : public FeatureExtractor {
public:
    SURFExtractor(std::unique_ptr <FeatureFinder> feature_finder_, std::shared_ptr <DescriptorDistance> dist_func_,
                  FeatureExtractorSettings settings_);

    void operator()( cv::InputArray image, cv::InputArray mask,
                     std::vector<cv::KeyPoint>& keypoints, std::vector<FeatureDescriptor> &descriptors);

    int GetLevels(){return 0;};
    float GetScaleFactor(){return 0.0;};
    std::vector<float> GetScaleFactors() { return std::vector<float>(); };
    std::vector<float> GetInverseScaleFactors(){ return std::vector<float>(); };
    std::vector<float> GetScaleSigmaSquares(){ return std::vector<float>(); };
    std::vector<float> GetInverseScaleSigmaSquares(){ return std::vector<float>(); };



private:
    std::unique_ptr<FeatureFinder> feature_finder;
    std::shared_ptr<DescriptorDistance> dist_func;
    FeatureExtractorSettings settings;

 //   int nlevels;

    std::vector<float> mvScaleFactor;


};
}

#endif //HYSLAM_SURFEXTRACTOR_H
