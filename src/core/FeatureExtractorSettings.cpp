
#include <FeatureExtractorSettings.h>

namespace HYSLAM{
    float FeatureExtractorSettings::determineSigma2(float feature_size){
        float scale_factor = feature_size/size_ref;
        return sigma_ref * (scale_factor*scale_factor);
    }
}

