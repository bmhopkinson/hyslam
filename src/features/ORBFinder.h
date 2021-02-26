//
// Created by cv-bhlab on 2/26/21.
//

#ifndef HYSLAM_ORBFINDER_H
#define HYSLAM_ORBFINDER_H

#include <FeatureFinder.h>

namespace HYSLAM {

class ORBFinder : public FeatureFinder {
public:
    ORBFinder() {};
    ORBFinder(double threshold_, bool non_max_suppression_);

    void setThreshold(double threshold_);
    double getThreshold();

    void setNonMaxSuppression(bool set){non_max_suppression = set;}

    void detect(cv::Mat image, std::vector<cv::KeyPoint> &keypoints);

private:
    int threshold = 20;
    bool non_max_suppression = true;

};

}// end namespace
#endif //HYSLAM_ORBFINDER_H
