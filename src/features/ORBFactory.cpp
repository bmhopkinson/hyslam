//
// Created by cv-bhlab on 3/16/21.
//

#include "ORBFactory.h"

#include <ORBVocabulary.h>
#include <ORBExtractor.h>
#include <ORBFinder.h>

namespace HYSLAM {

ORBFactory::ORBFactory() {
    // set default extractor and matcher settings;
    extractor_settings.nFeatures = 1000;
    extractor_settings.fScaleFactor = 1.2;
    extractor_settings.nLevels = 8;
    extractor_settings.N_CELLS = 30;
    extractor_settings.init_threshold = 20;
    extractor_settings.min_threshold = 4;

    matcher_settings.TH_HIGH = 100.0;
    matcher_settings.TH_LOW  = 50.0;

}

ORBFactory::ORBFactory(std::string settings_path_) : settings_path(settings_path_)
{
    LoadSettings(settings_path, "SLAM");
}

std::shared_ptr<FeatureExtractor> ORBFactory::getExtractor(std::string type) {
    LoadSettings(settings_path, type);
    return getExtractor(extractor_settings);
}

std::shared_ptr<FeatureExtractor> ORBFactory::getExtractor(FeatureExtractorSettings settings){
    std::shared_ptr<DescriptorDistance> dist_func = std::make_shared<ORBDistance>();
    return std::make_shared<ORBExtractor>(std::make_unique<ORBFinder>(20.0, true), dist_func, settings);
}

FeatureVocabulary* ORBFactory::getVocabulary(std::string type){
    LoadSettings(settings_path, type);
    return new ORBVocabulary(vocab_path);
}

std::shared_ptr<DescriptorDistance> ORBFactory::getDistanceFunc(){
    return std::make_shared<ORBDistance>();
}

FeatureExtractorSettings ORBFactory::getFeatureExtractorSettings(){
    return extractor_settings;
}

void ORBFactory::LoadSettings(std::string settings_path, std::string type){

    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    cv::FileNode extract = fSettings["ORB"][type]["Extractor"];
    extractor_settings.nFeatures = extract["N_Features"];
    extractor_settings.fScaleFactor = extract["scale_factor"];
    extractor_settings.nLevels = extract["N_Levels"];
    extractor_settings.N_CELLS = extract["N_Cells"];
    extractor_settings.init_threshold = extract["threshold_init"];
    extractor_settings.min_threshold = extract["threshold_min"];

    cv::FileNode match = fSettings["ORB"][type]["Matcher"];
    matcher_settings.TH_HIGH = match["threshold_high"];
    matcher_settings.TH_LOW  = match["threshold_low"];

    vocab_path = fSettings["ORB"][type]["Vocabulary"].string();

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << extractor_settings.nFeatures << std::endl;
    std::cout << "- Number of Cells: " << extractor_settings.N_CELLS << std::endl;
    std::cout << "- Scale Levels: " << extractor_settings.nLevels << std::endl;
    std::cout << "- Scale Factor: " << extractor_settings.fScaleFactor << std::endl;
    std::cout << "- Initial Extractor Threshold: " << extractor_settings.init_threshold << std::endl;
    std::cout << "- Minimum Extractor Threshold: " << extractor_settings.min_threshold << std::endl;

    std::cout << "matching threshold HIGH: " << matcher_settings.TH_HIGH << std::endl;
    std::cout << "matching threshold LOW: " << matcher_settings.TH_LOW<< std::endl;
    fSettings.release();

}





}