#include <TrackingStateNormal.h>
#include <Mapping.h> //necessary b/c of cyclic dependencies among main threads and TrackingState - work to eliminate
#include <Camera.h>
#include <Tracking_datastructs.h>

namespace HYSLAM {
TrackingStateNormal::TrackingStateNormal(optInfo optimizer_info_,StateNormalParameters params_, std::ofstream &log, MainThreadsStatus* thread_status_) :
    params(params_),  TrackingState(log, thread_status_)
{

    track_motion_model = std::make_unique<TrackMotionModel>(optimizer_info_, params.tmomo_params);
    track_reference_keyframe = std::make_unique<TrackReferenceKeyFrame>(optimizer_info_, params.trefkf_params);
    track_local_map = std::make_unique<TrackLocalMap>(optimizer_info_, params.tlocalmap_params);
}

bool TrackingStateNormal::initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories){
    int nmatches = 0;
    Camera camera = current_frame.getCamera();
    if(!trajectories.at(camera.camName)->getLastVelocityValid())
    {
       //std::cout << "trying to track with reference keyframe w/ no valid velocity: " << std::endl;
        nmatches = track_reference_keyframe->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );
    }
    else     //track with motion model - default when tracking is going smoothly
    {
       // std::cout << "trying to track with motion model: " << std::endl;
         nmatches = track_motion_model->track(current_frame, frames, pKF, pMap,trajectories.at("SLAM").get() );

        if(nmatches < params.thresh_init){
         //         std::cout << "tracking with motion model failed...will need to track with reference key frame." << std::endl;
            nmatches = track_reference_keyframe->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );
            (*pftracking) << "RefKF" << "\t";
        } else {
            (*pftracking) << "MoMo" << "\t";
        }
    }
    bool success = nmatches > params.thresh_init;
    (*pftracking) << success << "\t";
    return success;
}

bool TrackingStateNormal::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories){
    mnMatchesInliers =  track_local_map->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );
    (*pftracking) <<  mnMatchesInliers << "\t";
    return mnMatchesInliers > params.thresh_refine;
}

bool TrackingStateNormal::needNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force){
 //   if(mbOnlyTracking)
 //       return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
   // if(pLocalMapper->isStopped() || pLocalMapper->stopRequested())
   if(thread_status->mapping.isStopped() || thread_status->mapping.isStopRequested())
        return false;

    const int nKFs = pMap->KeyFramesInMap();

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = current_frame.mpReferenceKF->TrackedMapPoints(nMinObs);



    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    Camera camera = current_frame.getCamera();
    const ORBViews views = current_frame.getViews();
    if(camera.sensor !=0) //if not a monocular camera
    {
        for(int i =0; i<current_frame.N; i++)
        {
            if(views.depth(i)>0 && views.depth(i)<camera.thDepth)
            {
                if(current_frame.hasAssociation(i) && !current_frame.isOutlier(i))
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose < params.min_N_tracked_close) && ( nNonTrackedClose > params.thresh_N_nontracked_close);

    (*pftracking) << nRefMatches << "\t" << nTrackedClose << "\t" << nNonTrackedClose << "\t";

    // Thresholds
    float thRefRatio = params.min_frac_refKF_stereo;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(camera.sensor ==0) //if monocular
        thRefRatio = params.min_frac_refKF_mono;

    // Conditions used to decide on KF insertion
    //bool local_mapping_idle = pLocalMapper->AcceptKeyFrames();
    bool local_mapping_idle = thread_status->mapping.isAcceptingInput();
    bool max_interval_exceeded = current_frame.mnId>= last_keyframe_id+ params.max_KF_interval;
    bool min_interval_exceeded = current_frame.mnId>= last_keyframe_id+params.min_KF_interval;
    bool lack_close_landmarks =  (camera.sensor !=0) && bNeedToInsertClose;
    bool tracking_weak = mnMatchesInliers < (params.N_tracked_target - params.N_tracked_variance);
    bool tracking_dire = mnMatchesInliers < (params.N_tracked_target - 2*params.N_tracked_variance);

    bool definite_insert = (force || max_interval_exceeded || tracking_dire) ;
    bool optional_insert = min_interval_exceeded && (tracking_weak || lack_close_landmarks);

    bool insertKF = false;
    if(definite_insert || (optional_insert && local_mapping_idle))
    {
      //  std::cout << "KF insert requested: definite: " << definite_insert << ", tracking_dire: "<< tracking_dire <<", optional: " << optional_insert
       //         << ", tracking_weak: " << tracking_weak << ", mnMatchesInliers: " << mnMatchesInliers <<
        //        ", N_target: "<< params.N_tracked_target  << ", variance: "<< params.N_tracked_variance <<std::endl;
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
//        if(cam_cur != "SLAM") {std::cout << "attempting to insert keyframe" << std::endl;}
        if(local_mapping_idle)
        {
            insertKF = true;
        }
        else
        {
            //pLocalMapper->InterruptBA();
         //   thread_status->mapping.setInterrupt(true);
            if(camera.sensor !=0) //if not a monocular camera
            {
                if(pLocalMapper->KeyframesInQueue()<3) {
                    insertKF = true;
                }
                else {
                    insertKF = false;
                }
            }
            else {
                insertKF = false;
            }
        }
    }
    (*pftracking) << insertKF << "\t";
    return insertKF;
}

}