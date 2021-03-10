//
// Created by cv-bhlab on 2/26/21.
//

#ifndef HYSLAM_ORBFINDER_H
#define HYSLAM_ORBFINDER_H

#include <FeatureFinder.h>

namespace HYSLAM {

class ORBFinder : public FeatureFinder {
public:
    ORBFinder();
    ORBFinder(double threshold_, bool non_max_suppression_);

    void setThreshold(double threshold_);
    double getThreshold();

    void setNonMaxSuppression(bool set){non_max_suppression = set;}

    void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints);
    void compute(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat& descriptors);
    int descriptor_mat_type(){return 0; } // CV_8U : 0
    int descriptor_cols(){return 32;}  // ORB 256 bit descriptor (32 cols * 8 bits per col)

private:
    int threshold = 20;
    bool non_max_suppression = true;
    std::vector<cv::Point> pattern;
    std::vector<int> umax;

    void createPattern();
    void orientationSetup();
    void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax);
    void computeOrbDescriptor(const cv::KeyPoint& kpt, const cv::Mat& img, const cv::Point* pattern, uchar* desc);

};

}// end namespace
#endif //HYSLAM_ORBFINDER_H
