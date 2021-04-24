//
// Created by cv-bhlab on 3/15/21.
//

#ifndef HYSLAM_FEATUREVOCABULARY_H
#define HYSLAM_FEATUREVOCABULARY_H

/*
 * abstract base class for DBoW vocabularies for Feature Descriptors
 * Functionality:
 *  transform(const std::vector<FeatureDescriptor> &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector, int Nlevel )
 *      converts features into a Bag of Words descriptor based on the Features using vocabulary
 *      and feature_vector in which each entry in the vector represents a node in the vocabulary tree and the indexes of keypoint features contained in that node are listed - used for fast matching of visually similar features
 *  score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) - compares to BoW vectors from different images to determine how similar the image are - higher score = more similar
 */

#include <FeatureDescriptor.h>

#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#include <opencv2/opencv.hpp>

#include <vector>

namespace HYSLAM {

class FeatureVocabulary {
public:
    virtual void transform(const std::vector<FeatureDescriptor> &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector, int Nlevel ) = 0;
    virtual double score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const = 0;
    virtual unsigned int size() const = 0;
};

}
#endif //HYSLAM_FEATUREVOCABULARY_H
