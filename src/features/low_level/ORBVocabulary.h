//
// Created by cv-bhlab on 3/15/21.
//

#ifndef HYSLAM_ORBVOCABULARY_H
#define HYSLAM_ORBVOCABULARY_H

#include <FeatureVocabulary.h>

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

#include <memory>
#include <string>

namespace HYSLAM {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
        DBoW2_ORBVocabulary;

class ORBVocabulary : public FeatureVocabulary{
public:
    ORBVocabulary(std::string vocab_file);
    void transform(const std::vector<cv::Mat> &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector, int Nlevel );
    double score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const;
    unsigned int size() const;
private:
    std::unique_ptr<DBoW2_ORBVocabulary> orb_vocab;

};
}


#endif //HYSLAM_ORBVOCABULARY_H
