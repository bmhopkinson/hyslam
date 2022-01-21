#include <TrackMotionModel.h>
#include <FeatureMatcher.h>
#include <Optimizer.h>
#include <GenUtils.h>
#include <Camera.h>

#include <memory>

namespace HYSLAM{
TrackMotionModel::TrackMotionModel(optInfo optimizer_info_, const TrackMotionModelParameters &params_, FeatureFactory* factory)
: optimizer_info(optimizer_info_), params(params_), feature_factory(factory)
{}

int TrackMotionModel::track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* slam_trajectory){
    const Frame& last_frame = frames[0];
    const Frame& last_slamframe = frames[1];
    Camera camera = current_frame.getCamera();

   // FeatureMatcher matcher(params.match_nnratio, true);
   FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
   fm_settings.nnratio = params.match_nnratio;
   fm_settings.checkOri =true;
   feature_factory->setFeatureMatcherSettings(fm_settings);
   std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();

    cv::Mat Vcw;
    cv::Mat Tcw_cur;
    double delta_cur = current_frame.mTimeStamp - last_frame.mTimeStamp; //mVelocity is based on slam motion so need to reference time to that

    TrajectoryElement te = slam_trajectory->back();
    GenUtils::ScaleVelocity(te.Vcw, te.dt_vel, delta_cur,  Vcw);
    Tcw_cur =  Vcw * last_frame.mTcw;
    current_frame.SetPose(Tcw_cur);

    current_frame.clearAssociations();

    // Project points seen in previous frame
    int th;
    if(camera.sensor != 1) //1 = stereo camera
        th = params.match_radius_threshold_stereo;
    else
        th = params.match_radius_threshold_other;

    int nmatches = matcher->SearchByProjection(current_frame,last_frame, th,camera.sensor==0);

    // If few matches, uses a wider window search
    if(nmatches < params.N_min_matches)
    {
        current_frame.clearAssociations();
        th =  params.match_theshold_inflation_factor*th;
        nmatches = matcher->SearchByProjection(current_frame,last_frame, th,camera.sensor==0);
    }

    if(nmatches < params.N_min_matches){
        return -1;  //attempt to track failed
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&current_frame, optimizer_info);

    // Discard outliers
    int nmatchesMap = 0;
    const LandMarkMatches matches = current_frame.getLandMarkMatches();
    for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
        int LMid = it->first;
        MapPoint* pMP = it->second;
        if(!pMP){continue;}
        if(current_frame.isOutlier(LMid)){
            current_frame.removeLandMarkAssociation(LMid);
            nmatches--;
            //below is (should be) legacy
            current_frame.setOutlier(LMid, false);
            pMP->mnLastFrameSeen = current_frame.mnId;

        } else if (pMP->Observations() > 0)
        {
            nmatchesMap++;
        }
    }

    return nmatchesMap;

}
} //end namespace
