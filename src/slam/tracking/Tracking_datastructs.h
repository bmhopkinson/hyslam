#ifndef TRACKING_DATASTRUCTS_H_
#define TRACKING_DATASTRUCTS_H_

#include <opencv2/opencv.hpp>
#include "Frame.h"
#include <InitializerDataStructs.h>
#include <vector>
#include <deque>

/*
 * data structures related to tracking. most hold parameters for TrackingStrategies or TrackingStates
 */

namespace HYSLAM{

// Tracking states
enum class eTrackingState{
    SYSTEM_NOT_READY,
    NO_IMAGES_YET,
    INITIALIZATION,
    NORMAL,
    RELOCALIZATION,
    REINITIALIZE,
    NULL_STATE
};

class RANSACParameters{
public:
    double probability = 0.99;
    int minInliers = 8;
    int maxIterations = 300;
    int minSet = 4;
    float epsilon = 0.4;
    float th2 = 5.991;
};


// TRACKING STRATEGY PARAMETERS
class TrackLocalMapParameters{
public:
    TrackLocalMapParameters();
    TrackLocalMapParameters(cv::FileNode data);
    float match_nnratio = 0.8;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
    float match_radius_threshold = 5.0;  //scales radius within which a landmark is considered a potential match to a landmark_view
    int N_max_local_keyframes = 80; //maximum number of local keyframes making up the local map
    int N_neighbor_keyframes = 10; //number of neighbor keyframes to query when assembling the local map keyframes
};

class TrackMotionModelParameters{
public:
    TrackMotionModelParameters();
    TrackMotionModelParameters(cv::FileNode data);
    int N_min_matches = 20; //minimum number of matches for success
    float match_nnratio = 0.9;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
    float match_radius_threshold_stereo = 15.0;  //scales radius within which a landmark is considered a potential match to a landmark_view
    float match_radius_threshold_other = 7.0;  //scales radius within which a landmark is considered a potential match to a landmark_view
    float match_theshold_inflation_factor = 2.0; //factor by which radius threshold is raised if insufficient matches are obtained with intial threshold;
};

class TrackPlaceRecognitionParameters{
public:
    TrackPlaceRecognitionParameters();
    TrackPlaceRecognitionParameters(cv::FileNode data);
    float match_nnratio_1 = 0.75;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
    float match_nnratio_2 = 0.90;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
    int N_min_matches_BoW = 15; //minimum number of BoW matches for keyframe to be considered further.
    RANSACParameters ransac;
    int PnPsolver_iterations = 5;
    int N_min_matches_PoseOpt = 10; //minimum number of matches post pose optimization for keyframe to be considered further.
    int N_min_matches_success = 50; //minimum number of matches for succesful place recognition
    float match_radius_threshold_1 = 10;
    float match_radius_threshold_2 = 3;
    int ORBdist_1 = 100;
    int ORBdist_2 = 64;

private:
    void setDefaultRANSAC();
};

class TrackReferenceKeyFrameParameters {
public:
    TrackReferenceKeyFrameParameters();
    TrackReferenceKeyFrameParameters(cv::FileNode data);
    float match_nnratio = 0.7;  //factor by which best candidate landmark score with landmark_view must exceed 2nd best score to be considered a match
    int N_min_matches_BoW = 15; //minimum number of BoW matches for keyframe to be considered further.

};

//TRACKING STATE PARAMETERS
class StateNormalParameters{
public:
    StateNormalParameters();
    StateNormalParameters(cv::FileNode state_data,cv::FileNode strategy_data);
    TrackMotionModelParameters tmomo_params;
    TrackReferenceKeyFrameParameters trefkf_params;
    TrackLocalMapParameters tlocalmap_params;

    int thresh_init = 10; //number of tracked points initially needed
    int thresh_refine = 30; //number of tracked points required after refinement for success

    //keyframe insertion logic parameters
    int N_tracked_target = 200;
    int N_tracked_variance = 50;
    int min_KF_interval = 0;
    int max_KF_interval = 30;
    int min_N_tracked_close = 100;
    int thresh_N_nontracked_close = 70;
    float min_frac_refKF_mono = 0.9;
    float min_frac_refKF_stereo = 0.75;
};

class StateInitializeParameters{
public:
    StateInitializeParameters();
    StateInitializeParameters(cv::FileNode state_data,cv::FileNode strategy_data);
    MonoInitializerParameters mono_params;
    StereoInitializerParameters stereo_params;

    int N_min_mpts_monoslam =100; //minimum number of mappoint for initialization of monocular slam (doesn't apply when monocular is accessory cam)
};

class StateReInitializeParameters{
public:
    StateReInitializeParameters();
    StateReInitializeParameters(cv::FileNode state_data,cv::FileNode strategy_data);
    MonoInitializerParameters mono_params;
    StereoInitializerParameters stereo_params;
    int max_frames_elapsed = 5;

    int N_min_mpts_monoslam =100; //minimum number of mappoint for initialization of monocular slam (doesn't apply when monocular is accessory cam)
};

class StateRelocalizeParameters{
public:
    StateRelocalizeParameters();
    StateRelocalizeParameters(cv::FileNode state_data,cv::FileNode strategy_data);

    TrackPlaceRecognitionParameters tplacerecog_params;
    TrackLocalMapParameters tlocalmap_params;

    int thresh_init = 50; //number of tracked points initially needed
    int thresh_refine = 30; //number of tracked points required after refinement for success

};

class MonoInitialMatch
{
    //  std::vector<int> mvIniLastMatches;  //doesn't seem to be used
public:
    void clear();

    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;
};

class Velocity{
  public:
  //  Velocity();
    bool empty(){ return velocities.empty(); }
    void addElement(const cv::Mat vel, double &time, bool valid);
    int timeRange(const double t_start, const double t_stop, std::vector<cv::Mat> &vels, std::vector<double> &times);
    int integrate(const double t_start, const double t_stop, cv::Mat &vel_integrated);
    cv::Mat operator[](const int i);
    bool getLastValid() {return last_valid;}
    void ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled);   
    
  private:
    std::deque<cv::Mat> velocities; 
    std::deque<double> time_stamps;
    bool last_valid = false;
    int max_size = 120; // 2 seconds at 60 fps 


};


} //end namespace

#endif
