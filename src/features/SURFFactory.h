//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_SURFFACTORY_H
#define HYSLAM_SURFFACTORY_H

#include <FeatureFactory.h>

namespace HYSLAM {

class SURFFactory : public FeatureFactory {
public:
    FeatureExtractor *getExtractor(ORBextractorSettings settings);  //convert to unique_ptr
    FeatureVocabulary *getVocabulary(std::string file_name);
};

} //end namespace
#endif //HYSLAM_SURFFACTORY_H
