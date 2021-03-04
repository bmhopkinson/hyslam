#include <TrackMotionModel.h>
#include <ORBmatcher.h>
#include <Optimizer.h>
#include <GenUtils.h>
#include <Camera.h>

namespace HYSLAM{
TrackMotionModel::TrackMotionModel(optInfo optimizer_info_, const TrackMotionModelParameters &params_)
: optimizer_info(optimizer_info_), params(params_)
{}

int TrackMotionModel::track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* slam_trajectory){
    const Frame& last_frame = frames[0];
    const Frame& last_slamframe = frames[1];
    Camera camera = current_frame.getCamera();

    ORBmatcher matcher(params.match_nnratio, true);
    //  std::cout << "TrackwMotionModel, Frame: " << mCurrentFrame.mnId << std::endl;

    cv::Mat Vcw;
    cv::Mat Tcw_cur;
    if(camera.camName == "SLAM"){
        double delta_cur = current_frame.mTimeStamp - last_frame.mTimeStamp; //mVelocity is based on slam motion so need to reference time to that
      //  std::cout << "TrackMotionModel: dt " <<delta_cur << std::endl;
        TrajectoryElement te = slam_trajectory->back();
        GenUtils::ScaleVelocity(te.Vcw, te.dt_vel, delta_cur,  Vcw);  //scale 'velocity' based on time difference between that used to calculate the velocity and time betwween mono init frame, properly accounts for going back in time
        Tcw_cur =  Vcw * last_frame.mTcw;
     //   std::cout << "Velocity: " << Vcw << std::endl;
     //   std::cout << "pose estimate: " << Tcw_cur << std::endl;

    } else {  //THIS FAILS IF SLAM CAM HAS LOST TRACKING
        if(slam_trajectory->integrateVelocity( last_slamframe.mTimeStamp, last_frame.mTimeStamp, Vcw) == 0){
            cv::Mat Tcw_slam = Vcw*last_slamframe .mTcw;  //inferred position of SLAM camera at current frame
            Tcw_cur = camera.Tcam.inv() * Tcw_slam; //convert from position of SLAM camera to current camera

        }
        else {
            std::cout << "could not integrate velocity between: " << last_slamframe.mTimeStamp << " and " << last_frame.mTimeStamp << std::endl;
            return false;
        }

    }

    current_frame.SetPose(Tcw_cur);

    current_frame.clearAssociations();

    // Project points seen in previous frame
    int th;
    if(camera.sensor != 1) //1 = stereo camera
        th = params.match_radius_threshold_stereo;
    else
        th = params.match_radius_threshold_other;

    int nmatches = matcher.SearchByProjection(current_frame,last_frame, th,camera.sensor==0);
   // std::cout << "TrackMotionModel, SearchByProj matches" << nmatches << std::endl;

    //   std::cout << "TrackwMotionModel, Frame: " << mCurrentFrame.mnId << std::endl;
    //   mCurrentFrame.validateNewAssociations();
    // If few matches, uses a wider window search
    if(nmatches < params.N_min_matches)
    {
        current_frame.clearAssociations();
        th =  params.match_theshold_inflation_factor*th;
        nmatches = matcher.SearchByProjection(current_frame,last_frame, th,camera.sensor==0);
      //  std::cout << "TrackMotionModel, 2nd attempt to SearchByProjection, nmatches: " << nmatches << std::endl;
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
            //    pMP->mbTrackInView = false;
            pMP->mnLastFrameSeen = current_frame.mnId;

        } else if (pMP->Observations() > 0)
        {
            nmatchesMap++;
        }
    }

    return nmatchesMap;

}
} //end namespace
