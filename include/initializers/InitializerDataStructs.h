#ifndef INITIALIZERDATASTRUCTS_H_
#define INITIALIZERDATASTRUCTS_H_
#include <opencv2/opencv.hpp>

namespace HYSLAM {

class MonoInitializerParameters {
public:
    MonoInitializerParameters();
    MonoInitializerParameters(cv::FileNode data);
    int N_min_features = 100; //minimum number of features required to attempt initialization
    int N_min_matches = 100; //minimum number of matches required to attempt initialization
    float sigma = 1.0;// Standard Deviation and Variance
    int minTriangulated = 50; // min number of points triangulated
    float minFracTriangulated = 0.9;  //min fraction of matched points triangulated to consider initialization successful
    int MaxIterations = 200; // Ransac max iterations
    float match_nnratio = 0.9;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
};

class StereoInitializerParameters {
public:
    StereoInitializerParameters();
    StereoInitializerParameters(cv::FileNode data);
    int N_min_features = 300; //minimum number of features required for initialization
    int new_mpt_protection = 5; // protect new mappoint from culling for new_mpt_protection keyframes.
};

} //end namespace
#endif