//
// Created by cv-bhlab on 3/22/21.
//

#ifndef HYSLAM_FEATUREEXTRACTOR_H
#define HYSLAM_FEATUREEXTRACTOR_H

/*
 * abstract base class for high-level feature extractor
 * input is an image (mask is ignored currently) and outputs keypoints and their associated descriptors
 * only real functionality is that described above which is obtained through the overloaded () operator
 * rest of the th functions are legacy for working with ORB extractor and should be removed eventually.
 */

#include <FeatureExtractorSettings.h>
#include <FeatureFinder.h>
#include <FeatureDescriptor.h>
#include <DescriptorDistance.h>
#include <opencv/cv.h>

#include <memory>

namespace  HYSLAM {

class FeatureExtractor {
public:
virtual void operator()( cv::InputArray image, cv::InputArray mask,
                 std::vector<cv::KeyPoint>& keypoints, std::vector<FeatureDescriptor> &descriptors) = 0;

    virtual int GetLevels() =0;
    virtual float GetScaleFactor()=0;
    virtual std::vector<float> GetScaleFactors()=0;
    virtual std::vector<float> inline GetInverseScaleFactors()=0;
    virtual std::vector<float> inline GetScaleSigmaSquares()=0;
    virtual std::vector<float> inline GetInverseScaleSigmaSquares()=0;

};

}
#endif //HYSLAM_FEATUREEXTRACTOR_H
