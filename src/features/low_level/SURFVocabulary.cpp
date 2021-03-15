//
// Created by cv-bhlab on 3/15/21.
//

#include "SURFVocabulary.h"
namespace HYSLAM {

SURFVocabulary::SURFVocabulary(std::string vocab_file){
    vocab = std::make_unique<Surf64Vocabulary>(vocab_file);
}

void SURFVocabulary::transform(const std::vector<FeatureDescriptor>  &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector,
          int Nlevel){
    std::vector<std::vector<float>> features_raw;
    features_raw.reserve(features.size());
    for(auto it = features.cbegin(); it != features.cend(); ++it){
        std::vector<float> temp;
        cv::Mat desc = it->rawDescriptor();
        temp.assign((float*)desc.datastart, (float*)desc.dataend);
        features_raw.push_back(temp);
    }
    vocab->transform(features_raw,BoW_vector, feature_vector, Nlevel );
}

double SURFVocabulary::score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const{
    return vocab->score(a, b);
}

unsigned int SURFVocabulary::size() const{
    return vocab->size();
}

} //end namespace