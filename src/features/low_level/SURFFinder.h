//
// Created by cv-bhlab on 3/2/21.
//

#ifndef HYSLAM_SURFFINDER_H
#define HYSLAM_SURFFINDER_H

/*
 *  implements FeatureFinder for SURF (speeded up robust features) features
 *  see FeatureFinder for functionality
 *  the class is just a wrapper to cv::xfeatures2d::SURF
 */

#include <FeatureFinder.h>
#include <FeatureExtractorSettings.h>
#include "opencv2/xfeatures2d.hpp"

namespace HYSLAM {
class SURFFinder : public FeatureFinder {
public:
    SURFFinder();
    SURFFinder(FeatureExtractorSettings settings);
    SURFFinder(double threshold_);
    void setThreshold(double threshold_);
    double getThreshold(){return threshold;};
    void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints); //detect features
    void compute(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors); //compute descriptors
    int descriptor_mat_type(){return 5; } // CV_32F : 5
    int descriptor_cols(){return 64;}  //SURF 64 element descriptor

    int getNOctaves() const;
    void setNOctaves(int nOctaves);

private:
    double threshold = 400;
    int nOctaves = 4;

    int nOctaveLayers = 3;
    cv::Ptr<cv::xfeatures2d::SURF> detector;
};

} //end namespace
#endif //HYSLAM_SURFFINDER_H
