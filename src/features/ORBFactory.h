//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_ORBFACTORY_H
#define HYSLAM_ORBFACTORY_H

/*
 * factory class that creates and returns classes to extract and match ORB Features
 */

#include <FeatureFactory.h>
#include <FeatureExtractorSettings.h>

#include <string>

namespace HYSLAM {

class ORBFactory : public FeatureFactory {
public:
    ORBFactory();
    ORBFactory(std::string settings_path_);
   // FeatureExtractor* getExtractor(); //uses current feature extractor settings
    FeatureExtractor* getExtractor(std::string type);
    FeatureExtractor* getExtractor(FeatureExtractorSettings settings_);  //specify desired extractor settings. convert return type to unique_ptr
    FeatureVocabulary* getVocabulary(std::string type);
 //   FeatureVocabulary* getVocabulary(); //default
    std::shared_ptr<DescriptorDistance> getDistanceFunc();
    FeatureExtractorSettings getFeatureExtractorSettings();

private:
    FeatureExtractorSettings extractor_settings;
    std::string vocab_path;
    std::string settings_path;

    void LoadSettings(std::string settings_path, std::string type);
};

}


#endif //HYSLAM_ORBFACTORY_H
