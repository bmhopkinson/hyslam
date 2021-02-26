#ifndef MAPPINGDATASTRUCTS_H_
#define MAPPINGDATASTRUCTS_H_

#include <opencv2/opencv.hpp>

namespace HYSLAM{

class ProcessNewKeyFrameParameters{  //placeholder
public:
    ProcessNewKeyFrameParameters(){};
    ProcessNewKeyFrameParameters(cv::FileNode data){};
};

class KeyFrameCullerParameters{
public:
    KeyFrameCullerParameters(){};
    KeyFrameCullerParameters(cv::FileNode data);
    int LMobservations_thresh = 3;  //min number of land mark observations needed for redundancy consideration
    float frac_redundant = 0.9; //fraction of redundant mappoints in a KF that leads to its culling
};

class LandMarkCullerParameters {
public:
    LandMarkCullerParameters(){};
    LandMarkCullerParameters(cv::FileNode data);
    int min_lm_observations_mono = 2;
    int min_lm_observations_stereo = 3;
    int KF_grace_period = 2;

};

class LandMarkTriangulatorParameters{
public:
    LandMarkTriangulatorParameters(){};
    LandMarkTriangulatorParameters(cv::FileNode data);
    float match_nnratio = 0.6;
    int N_neighborKFs_mono = 20;
    int N_neighborKFs_stereo = 10;
    float ratio_factor =  1.5;
    float min_baseline_depth_ratio = 0.010;
    float error_factor_mono = 5.99;
    float error_factor_stereo = 7.80;
};

class LandMarkFuserParameters {
public:
    LandMarkFuserParameters(){};
    LandMarkFuserParameters(cv::FileNode data);
    int N_neighborKFs_mono = 20;
    int N_neighborKFs_stereo = 10;
    int N_secondNeighbors = 5;
};

} //end namespace


#endif