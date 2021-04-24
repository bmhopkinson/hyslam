//
// Created by cv-bhlab on 3/15/21.
//

#ifndef HYSLAM_SURFVOCABULARY_H
#define HYSLAM_SURFVOCABULARY_H

/*
 * implements FeatureVocabulary for SURF Features
 * a wrapper class for DBoW2 SURFVocabulary
 */

#include <FeatureVocabulary.h>

#include "DBoW2/FSurf64.h"
#include "DBoW2/TemplatedVocabulary.h"

#include <memory>
#include <string>

namespace HYSLAM {
typedef DBoW2::TemplatedVocabulary<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64>
        Surf64Vocabulary;

class SURFVocabulary : public FeatureVocabulary  {
public:
    SURFVocabulary(std::string vocab_file);
    void transform(const std::vector<FeatureDescriptor>  &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector, int Nlevel );
    double score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const;
    unsigned int size() const;
private:
 std::unique_ptr<Surf64Vocabulary> vocab;
};

}//end namespace

#endif //HYSLAM_SURFVOCABULARY_H
