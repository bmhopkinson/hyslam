//
// Created by cv-bhlab on 3/16/21.
//

#include "SURFFactory.h"

#include <SURFVocabulary.h>
#include <SURFExtractor.h>
#include <SURFFinder.h>

namespace HYSLAM {

SURFFactory::SURFFactory() {    //default values for extractor and matcher

    extractor_settings.nFeatures = 1000;
    extractor_settings.nLevels = 5;
    extractor_settings.init_threshold = 600;
    extractor_settings.min_threshold = 400;
    extractor_settings.N_CELLS = 2;

    matcher_settings.TH_HIGH = 1.5;
    matcher_settings.TH_LOW  = 0.75;

}

SURFFactory::SURFFactory(std::string settings_path_) : settings_path(settings_path_){
    LoadSettings(settings_path, "SLAM");
}

std::shared_ptr<FeatureExtractor> SURFFactory::getExtractor(std::string type){
    LoadSettings(settings_path, type);
    return getExtractor(extractor_settings);
}

std::shared_ptr<FeatureExtractor> SURFFactory::getExtractor(FeatureExtractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<SURFDistance>();
    return std::make_shared<SURFExtractor>(std::make_unique<SURFFinder>(settings), dist_func, settings);
}

FeatureVocabulary* SURFFactory::getVocabulary(std::string type){
    LoadSettings(settings_path, type);
    return new SURFVocabulary(vocab_path);
}

std::shared_ptr<DescriptorDistance> SURFFactory::getDistanceFunc(){
    return std::make_shared<SURFDistance>();
}

FeatureExtractorSettings SURFFactory::getFeatureExtractorSettings(){
    return extractor_settings;
}

void SURFFactory::LoadSettings(std::string settings_path, std::string type)
{
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    cv::FileNode extract = fSettings["SURF"][type]["Extractor"];
    extractor_settings.nFeatures = extract["N_Features"];
    extractor_settings.nLevels = extract["N_Levels"];
    extractor_settings.init_threshold = extract["threshold_init"];
    extractor_settings.min_threshold = extract["threshold_min"];
    extractor_settings.N_CELLS = extract["N_Cells"];

    cv::FileNode match = fSettings["SURF"][type]["Matcher"];
    matcher_settings.TH_HIGH = match["threshold_high"];
    matcher_settings.TH_LOW  = match["threshold_low"];


    vocab_path = fSettings["SURF"][type]["Vocabulary"].string();

    std::cout << std::endl  << "SURF Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << extractor_settings.nFeatures << std::endl;
    std::cout << "- Scale Levels: " << extractor_settings.nLevels << std::endl;
    std::cout << "- Scale Factor: " << extractor_settings.fScaleFactor << std::endl;
    std::cout << "- Initial Extractor Threshold: " << extractor_settings.init_threshold << std::endl;
    std::cout << "- Minimum Extractor Threshold: " << extractor_settings.min_threshold << std::endl;

    std::cout << "matching threshold HIGH: " << matcher_settings.TH_HIGH << std::endl;
    std::cout << "matching threshold LOW: " << matcher_settings.TH_LOW<< std::endl;
    fSettings.release();

}



}