//
// Created by cv-bhlab on 3/15/21.
//

#include "ORBVocabulary.h"

namespace HYSLAM {

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

ORBVocabulary::ORBVocabulary(std::string vocab_file){
    clock_t tStart = clock();
    orb_vocab = std::make_unique<DBoW2_ORBVocabulary>();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(vocab_file, ".txt"))
        bVocLoad = orb_vocab->loadFromTextFile(vocab_file);
    else
        bVocLoad = orb_vocab->loadFromBinaryFile(vocab_file);
    if(!bVocLoad)
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Failed to open at: " << vocab_file << std::endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void ORBVocabulary::transform(const std::vector<cv::Mat> &features, DBoW2::BowVector &BoW_vector, DBoW2::FeatureVector &feature_vector,
          int Nlevel)
{
    orb_vocab->transform(features,BoW_vector, feature_vector, Nlevel );

}

double ORBVocabulary::score(const DBoW2::BowVector &a, const DBoW2::BowVector &b) const{
    return orb_vocab->score(a, b);
}

unsigned int ORBVocabulary::size() const {
    return orb_vocab->size();

}

}//end namespace