
#ifndef HYSLAM_FEATUREEXTRACTORSETTINGS_H
#define HYSLAM_FEATUREEXTRACTORSETTINGS_H

namespace HYSLAM{

class FeatureExtractorSettings{
public:
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    int init_threshold;
    int min_threshold;
    float size_ref = 31;
    float sigma_ref = 1.0;
    int N_CELLS;

    float determineSigma2(float feature_size);
private:

};

}//close namespace

#endif