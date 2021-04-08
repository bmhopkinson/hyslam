//
// Created by cv-bhlab on 3/16/21.
//

#include "SURFFactory.h"

#include <SURFVocabulary.h>
#include <SURFExtractor.h>
#include <SURFFinder.h>

namespace HYSLAM {

SURFFactory::SURFFactory(std::string settings_path){
    LoadSettings(settings_path, extractor_settings, matcher_settings);
}

FeatureExtractor* SURFFactory::getExtractor(){
    return getExtractor(extractor_settings);
}

FeatureExtractor* SURFFactory::getExtractor(FeatureExtractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<SURFDistance>();
    return new SURFExtractor(std::make_unique<SURFFinder>(settings), dist_func, settings);
}

FeatureVocabulary* SURFFactory::getVocabulary(std::string file_name){
    return new SURFVocabulary(file_name);
}

FeatureVocabulary* SURFFactory::getVocabulary(){
    return getVocabulary(vocab_path);
}

std::shared_ptr<DescriptorDistance> SURFFactory::getDistanceFunc(){
    return std::make_shared<SURFDistance>();
}

FeatureExtractorSettings SURFFactory::getFeatureExtractorSettings(){
    return extractor_settings;
}

void SURFFactory::LoadSettings(std::string settings_path, FeatureExtractorSettings &extractor_settings_, FeatureMatcherSettings &matcher_settings_){
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    cv::FileNode extract = fSettings["SURF"]["Extractor"];
    extractor_settings_.nFeatures = extract["N_Features"];
    extractor_settings_.nLevels = extract["N_Levels"];
    extractor_settings_.init_threshold = extract["threshold_init"];
    extractor_settings_.min_threshold = extract["threshold_min"];
    extractor_settings_.N_CELLS = extract["N_Cells"];

    cv::FileNode match = fSettings["SURF"]["Matcher"];
    matcher_settings_.TH_HIGH = match["threshold_high"];
    matcher_settings_.TH_LOW  = match["threshold_low"];


    vocab_path = fSettings["SURF"]["Vocabulary"].string();

    std::cout << std::endl  << "SURF Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << extractor_settings_.nFeatures << std::endl;
    std::cout << "- Scale Levels: " << extractor_settings_.nLevels << std::endl;
    std::cout << "- Scale Factor: " << extractor_settings_.fScaleFactor << std::endl;
    std::cout << "- Initial Extractor Threshold: " << extractor_settings_.init_threshold << std::endl;
    std::cout << "- Minimum Extractor Threshold: " << extractor_settings_.min_threshold << std::endl;

    std::cout << "matching threshold HIGH: " << matcher_settings_.TH_HIGH << std::endl;
    std::cout << "matching threshold LOW: " << matcher_settings_.TH_LOW<< std::endl;
    fSettings.release();

}

}