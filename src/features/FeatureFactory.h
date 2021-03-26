//
// Created by cv-bhlab on 3/16/21.
//

#ifndef HYSLAM_FEATUREFACTORY_H
#define HYSLAM_FEATUREFACTORY_H

#include <FeatureExtractor.h>
#include <FeatureVocabulary.h>
#include <FeatureMatcher.h>

#include <string>
#include <memory>

namespace HYSLAM {

class FeatureFactory {
public:
    virtual FeatureExtractor* getExtractor() = 0;  //convert to unique_ptr
    virtual FeatureExtractor* getExtractor(FeatureExtractorSettings settings) = 0;  //convert to unique_ptr
    virtual FeatureVocabulary* getVocabulary(std::string file_name) =0 ;
    virtual FeatureVocabulary* getVocabulary() =0 ;
    virtual std::shared_ptr<DescriptorDistance> getDistanceFunc() = 0;
    virtual FeatureExtractorSettings getFeatureExtractorSettings() = 0;
    std::unique_ptr<FeatureMatcher> getFeatureMatcher();
    FeatureMatcherSettings getFeatureMatcherSettings() const {return matcher_settings; } ;
    void setFeatureMatcherSettings(FeatureMatcherSettings fm_settings){matcher_settings = fm_settings;};
protected:
    FeatureMatcherSettings matcher_settings;
};
}


#endif //HYSLAM_FEATUREFACTORY_H
