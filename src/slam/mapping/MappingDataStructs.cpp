#include <MappingDataStructs.h>

namespace HYSLAM{

KeyFrameCullerParameters::KeyFrameCullerParameters(cv::FileNode data){
    LMobservations_thresh = data["LMobservations_thresh"];
    frac_redundant = data["frac_redundant"];
}

LandMarkCullerParameters::LandMarkCullerParameters(cv::FileNode data) {
    min_lm_observations_mono = data["min_lm_observations_mono"];
    min_lm_observations_stereo =  data["min_lm_observations_stereo"];
    KF_grace_period =  data["KF_grace_period"];
}

LandMarkTriangulatorParameters::LandMarkTriangulatorParameters(cv::FileNode data) {
    match_nnratio = data["match_nnratio"];
    N_neighborKFs_mono = data["N_neighborKFs_mono"];
    N_neighborKFs_stereo = data["N_neighborKFs_stereo"];
    ratio_factor = data["ratio_factor"];
    min_baseline_depth_ratio = data["min_baseline_depth_ratio"];
    error_factor_mono = data["error_factor_mono"];
    error_factor_stereo = data["error_factor_stereo"];
}

LandMarkFuserParameters::LandMarkFuserParameters(cv::FileNode data) {
    N_neighborKFs_mono = data["N_neighborKFs_mono"];
    N_neighborKFs_stereo = data["N_neighborKFs_stereo"];
    N_secondNeighbors = data["N_secondNeighbors"];
}

}