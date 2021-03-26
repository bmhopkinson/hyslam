//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_ORBFACTORY_H
#define HYSLAM_ORBFACTORY_H

#include <FeatureFactory.h>
#include <FeatureExtractorSettings.h>

#include <string>

namespace HYSLAM {

class ORBFactory : public FeatureFactory {
public:
    ORBFactory(std::string settings_path);
    FeatureExtractor* getExtractor(); //uses current feature extractor settings
    FeatureExtractor* getExtractor(FeatureExtractorSettings settings_);  //specify desired extractor settings. convert return type to unique_ptr
    FeatureVocabulary* getVocabulary(std::string file_name);
    FeatureVocabulary* getVocabulary(); //default
    std::shared_ptr<DescriptorDistance> getDistanceFunc();
    FeatureExtractorSettings getFeatureExtractorSettings();

private:
    FeatureExtractorSettings extractor_settings;
    std::string vocab_path;

    void LoadSettings(std::string settings_path, FeatureExtractorSettings &extractor_settings_, FeatureMatcherSettings &matcher_settings_);
};

}


#endif //HYSLAM_ORBFACTORY_H
