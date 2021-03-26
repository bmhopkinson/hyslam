//
// Created by cv-bhlab on 3/16/21.
//

#include "FeatureFactory.h"
namespace HYSLAM{
std::unique_ptr<FeatureMatcher> FeatureFactory::getFeatureMatcher(){
    return std::make_unique<FeatureMatcher>(matcher_settings);
}
}