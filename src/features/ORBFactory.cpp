//
// Created by cv-bhlab on 3/16/21.
//

#include "ORBFactory.h"

#include <ORBVocabulary.h>
#include <ORBExtractor.h>
#include <ORBFinder.h>

namespace HYSLAM {

ORBFactory::ORBFactory(std::string settings_path){
    LoadSettings(settings_path, extractor_settings, matcher_settings);
}

FeatureExtractor* ORBFactory::getExtractor(){
    return getExtractor(extractor_settings);
}

FeatureExtractor* ORBFactory::getExtractor(FeatureExtractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<ORBDistance>();
    return new ORBExtractor(std::make_unique<ORBFinder>(20.0, true), dist_func, settings);
}

FeatureVocabulary* ORBFactory::getVocabulary(std::string file_name){
 return new ORBVocabulary(file_name);
}

FeatureVocabulary* ORBFactory::getVocabulary(){
    return getVocabulary(vocab_path);
}

std::shared_ptr<DescriptorDistance> ORBFactory::getDistanceFunc(){
    return std::make_shared<ORBDistance>();
}

FeatureExtractorSettings ORBFactory::getFeatureExtractorSettings(){
    return extractor_settings;
}

void ORBFactory::LoadSettings(std::string settings_path, FeatureExtractorSettings &extractor_settings_, FeatureMatcherSettings &matcher_settings_){

    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    cv::FileNode extract = fSettings["ORB"]["Extractor"];
    extractor_settings_.nFeatures = extract["N_Features"];
    extractor_settings_.fScaleFactor = extract["scale_factor"];
    extractor_settings_.nLevels = extract["N_Levels"];
    extractor_settings_.N_CELLS = extract["N_Cells"];
    extractor_settings_.init_threshold = extract["threshold_init"];
    extractor_settings_.min_threshold = extract["threshold_min"];

    cv::FileNode match = fSettings["ORB"]["Matcher"];
    matcher_settings_.TH_HIGH = match["threshold_high"];
    matcher_settings_.TH_LOW  = match["threshold_low"];

    vocab_path = fSettings["ORB"]["Vocabulary"].string();

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << extractor_settings_.nFeatures << std::endl;
    std::cout << "- Number of Cells: " << extractor_settings_.N_CELLS << std::endl;
    std::cout << "- Scale Levels: " << extractor_settings_.nLevels << std::endl;
    std::cout << "- Scale Factor: " << extractor_settings_.fScaleFactor << std::endl;
    std::cout << "- Initial Extractor Threshold: " << extractor_settings_.init_threshold << std::endl;
    std::cout << "- Minimum Extractor Threshold: " << extractor_settings_.min_threshold << std::endl;

    std::cout << "matching threshold HIGH: " << matcher_settings_.TH_HIGH << std::endl;
    std::cout << "matching threshold LOW: " << matcher_settings_.TH_LOW<< std::endl;
    fSettings.release();

}

}