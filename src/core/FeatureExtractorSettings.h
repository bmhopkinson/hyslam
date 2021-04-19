
#ifndef HYSLAM_FEATUREEXTRACTORSETTINGS_H
#define HYSLAM_FEATUREEXTRACTORSETTINGS_H

/*
 * data structure representing settings used for FeatureExtractors - right now geared toward classic point features (e.g. ORB, SURF, SIFT)
 * nFeatures: target number of features to extract (NOT necessarily the number actually extracted)
 * fScaleFactor is scale factor between levels in image pyramid
 * nLevels number of levels in feature pyramid
 * thresholds are feature detector response thresholds (init = preferred higher threshold, min = lowest allowable)
 * size_ref = reference feature size (in pixels) to base scaling of error term (sigma_ref) on.
 * sigma_ref = error estimate for a feature of size "size_ref"
 * N_CELLS = number of cols and rows to break image into prior to feature extraction - helps distribute features across the image
 * determineSigma2(float feature_size) - calculates error estimate for a feature of size feature_size based on sigma_ref and size_ref
 */

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