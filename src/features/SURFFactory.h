//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_SURFFACTORY_H
#define HYSLAM_SURFFACTORY_H

/*
 * factory class that creates and returns classes to extract and match SURF Features
 */

#include <FeatureFactory.h>

namespace HYSLAM {

class SURFFactory : public FeatureFactory {
public:
    SURFFactory();
    SURFFactory(std::string settings_path_);
    std::shared_ptr<FeatureExtractor> getExtractor(std::string type);
    std::shared_ptr<FeatureExtractor> getExtractor(FeatureExtractorSettings settings_);  //specify desired extractor settings. convert return type to unique_ptr
    FeatureVocabulary* getVocabulary(std::string type);
    std::shared_ptr<DescriptorDistance> getDistanceFunc();
    FeatureExtractorSettings getFeatureExtractorSettings();

private:
    FeatureExtractorSettings extractor_settings;
    std::string vocab_path;
    std::string settings_path;

    void LoadSettings(std::string settings_path, std::string type);
};

} //end namespace
#endif //HYSLAM_SURFFACTORY_H
