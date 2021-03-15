//
// Created by cv-bhlab on 3/15/21.
//

#ifndef HYSLAM_FEATUREVOCABULARY_H
#define HYSLAM_FEATUREVOCABULARY_H

#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#include <opencv2/opencv.hpp>

#include <vector>

namespace HYSLAM {

class FeatureVocabulary {
public:
    virtual void transform(const std::vector<cv::Mat> &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector, int Nlevel ) = 0;
    virtual double score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const = 0;
    virtual unsigned int size() const = 0;
};

}
#endif //HYSLAM_FEATUREVOCABULARY_H
