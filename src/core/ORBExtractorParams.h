//
// Created by cv-bhlab on 3/9/21.
//

#ifndef HYSLAM_ORBEXTRACTORPARAMS_H
#define HYSLAM_ORBEXTRACTORPARAMS_H

#include <vector>

namespace HYSLAM {

struct ORBExtractorParams {
    ORBExtractorParams(){};
 //   ORBExtractorParams(FeatureExtractor* extractor);
  //  void setParams(FeatureExtractor* extractor);
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    float size_ref = 31;
    float sigma_ref = 1.0;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvInvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    float determineSigma2(float feature_size);
};
}


#endif //HYSLAM_ORBEXTRACTORPARAMS_H
