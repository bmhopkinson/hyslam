//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_SURFFACTORY_H
#define HYSLAM_SURFFACTORY_H

#include <FeatureFactory.h>

namespace HYSLAM {

class SURFFactory : public FeatureFactory {
public:
    SURFFactory(std::string settings_path);
    FeatureExtractor* getExtractor(); //uses current feature extractor settings
    FeatureExtractor *getExtractor(FeatureExtractorSettings settings);  //convert to unique_ptr
    FeatureVocabulary *getVocabulary(std::string file_name);
    FeatureVocabulary* getVocabulary(); //default
    std::shared_ptr<DescriptorDistance> getDistanceFunc();
    FeatureExtractorSettings getFeatureExtractorSettings();

private:
    FeatureExtractorSettings extractor_settings;
    std::string vocab_path;

    void LoadSettings(std::string settings_path, FeatureExtractorSettings &extractor_settings_, FeatureMatcherSettings &matcher_settings_);
};

} //end namespace
#endif //HYSLAM_SURFFACTORY_H
