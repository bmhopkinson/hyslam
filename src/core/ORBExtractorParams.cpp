//
// Created by cv-bhlab on 3/9/21.
//

#include "ORBExtractorParams.h"
namespace HYSLAM{
    float ORBExtractorParams::determineSigma2(float feature_size){
        float scale_factor = feature_size/size_ref;
        return sigma_ref * (scale_factor*scale_factor);
    }
}