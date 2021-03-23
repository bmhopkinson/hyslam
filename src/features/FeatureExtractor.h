//
// Created by cv-bhlab on 3/22/21.
//

#ifndef HYSLAM_FEATUREEXTRACTOR_H
#define HYSLAM_FEATUREEXTRACTOR_H

#include <FeatureFinder.h>
#include <FeatureDescriptor.h>
#include <DescriptorDistance.h>
#include <opencv/cv.h>

#include <memory>

namespace  HYSLAM {

class FeatureExtractorSettings{
public:
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    int init_threshold;
    int min_threshold;
private:

};

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
