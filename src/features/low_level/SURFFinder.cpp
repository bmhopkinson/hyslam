//
// Created by cv-bhlab on 3/2/21.
//

#include "SURFFinder.h"

namespace HYSLAM {

SURFFinder::SURFFinder(){
    detector = cv::xfeatures2d::SURF::create(threshold, nOctaves, nOctaveLayers);
}

SURFFinder::SURFFinder(FeatureExtractorSettings settings){
    threshold = settings.init_threshold;
    nOctaves = settings.nLevels;
    detector = cv::xfeatures2d::SURF::create(threshold, nOctaves, nOctaveLayers);
}

SURFFinder::SURFFinder(double threshold_) : threshold(threshold_)
{
    detector = cv::xfeatures2d::SURF::create(threshold, nOctaves, nOctaveLayers);
}

void SURFFinder::setThreshold(double threshold_){
    detector->setHessianThreshold(threshold_);
    threshold = threshold_;
}
void SURFFinder::detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints) {
    detector->detect(image, keypoints);
}

void SURFFinder::compute(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) {
    detector->compute(image, keypoints, descriptors);
}

int SURFFinder::getNOctaves() const {
    return nOctaves;
}

void SURFFinder::setNOctaves(int nOctaves) {
    SURFFinder::nOctaves = nOctaves;
}

}
