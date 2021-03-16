//
// Created by cv-bhlab on 3/16/21.
//

#include "SURFFactory.h"

#include <SURFVocabulary.h>
#include <FeatureExtractor.h>
#include <SURFFinder.h>

namespace HYSLAM {

FeatureExtractor* SURFFactory::getExtractor(ORBextractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<SURFDistance>();
    return new FeatureExtractor(std::make_unique<SURFFinder>(), dist_func, settings);
}

FeatureVocabulary* SURFFactory::getVocabulary(std::string file_name){
    return new SURFVocabulary(file_name);
}

}