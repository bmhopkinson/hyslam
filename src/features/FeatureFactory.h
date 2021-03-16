//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_FEATUREFACTORY_H
#define HYSLAM_FEATUREFACTORY_H

#include <FeatureExtractor.h>
#include <FeatureVocabulary.h>

#include <string>

namespace HYSLAM {

class FeatureFactory {
public:
    virtual FeatureExtractor* getExtractor(ORBextractorSettings settings) = 0;  //convert to unique_ptr
    virtual FeatureVocabulary* getVocabulary(std::string file_name) =0 ;
};
}


#endif //HYSLAM_FEATUREFACTORY_H
