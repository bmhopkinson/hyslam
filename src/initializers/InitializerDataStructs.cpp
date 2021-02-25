#include <InitializerDataStructs.h>

namespace ORB_SLAM2{

MonoInitializerParameters::MonoInitializerParameters(){}

MonoInitializerParameters::MonoInitializerParameters(cv::FileNode data){
    N_min_features = data["N_min_features"];
    N_min_matches = data["N_min_matches"];
    sigma = data["sigma"];
    minTriangulated = data["minTriangulated"];
    minFracTriangulated = data["minFracTriangulated"];
    MaxIterations = data["MaxIterations"];
    match_nnratio = data["match_nnratio"];
}


StereoInitializerParameters::StereoInitializerParameters(){}
StereoInitializerParameters::StereoInitializerParameters(cv::FileNode data){
    N_min_features = data["N_min_features"]; //minimum number of features required for initialization
    new_mpt_protection = data["new_mpt_protection"]; // protect new mappoint from culling for new_mpt_protection keyframes.
}

}