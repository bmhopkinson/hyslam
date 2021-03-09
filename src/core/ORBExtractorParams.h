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
    std::vector<float> mvScaleFactors;
    std::vector<float> mvInvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};
}


#endif //HYSLAM_ORBEXTRACTORPARAMS_H
