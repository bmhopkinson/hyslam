//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_ORBFACTORY_H
#define HYSLAM_ORBFACTORY_H

#include <FeatureFactory.h>

namespace HYSLAM {

class ORBFactory : public FeatureFactory {
public:
    FeatureExtractor* getExtractor(ORBextractorSettings settings);  //convert to unique_ptr
    FeatureVocabulary* getVocabulary(std::string file_name);
};

}


#endif //HYSLAM_ORBFACTORY_H
