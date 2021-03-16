//
// Created by cv-bhlab on 3/16/21.
//

#include "ORBFactory.h"

#include <ORBVocabulary.h>
#include <FeatureExtractor.h>
#include <ORBFinder.h>

namespace HYSLAM {
FeatureExtractor* ORBFactory::getExtractor(ORBextractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<ORBDistance>();
    return new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), dist_func, settings);
}

FeatureVocabulary* ORBFactory::getVocabulary(std::string file_name){
 return new ORBVocabulary(file_name);
}

}