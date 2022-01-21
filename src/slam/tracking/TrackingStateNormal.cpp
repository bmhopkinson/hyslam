#include <TrackingStateNormal.h>
#include <Camera.h>
#include <Tracking_datastructs.h>

namespace HYSLAM {

int TrackingStateNormal::n_calls =0;
std::chrono::duration<int, std::milli> TrackingStateNormal::init_pose_duration;
std::chrono::duration<int, std::milli> TrackingStateNormal::refine_pose_duration;

TrackingStateNormal::TrackingStateNormal(optInfo optimizer_info_,StateNormalParameters params_, std::ofstream &log,
                                         MainThreadsStatus* thread_status_, FeatureFactory* factory) :
    params(params_), feature_factory(factory), TrackingState(log, thread_status_)
{

    track_motion_model = std::make_unique<TrackMotionModel>(optimizer_info_, params.tmomo_params, feature_factory);
    track_reference_keyframe = std::make_unique<TrackReferenceKeyFrame>(optimizer_info_, params.trefkf_params, feature_factory);
    track_local_map = std::make_unique<TrackLocalMap>(optimizer_info_, params.tlocalmap_params, feature_factory);
}

bool TrackingStateNormal::initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories){
    n_calls++;
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    int nmatches = 0;
    Camera camera = current_frame.getCamera();
    if(!trajectories.at(camera.camName)->getLastVelocityValid())
    {
      nmatches = track_reference_keyframe->track(current_frame, frames, pKF, pMap, trajectories.at(camera.camName).get() );
    }
    else     //track with motion model - default when tracking is going smoothly
    {
     nmatches = track_motion_model->track(current_frame, frames, pKF, pMap,trajectories.at(camera.camName).get() );

        if(nmatches < params.thresh_init){
            nmatches = track_reference_keyframe->track(current_frame, frames, pKF, pMap, trajectories.at(camera.camName).get() );
            (*pftracking) << "RefKF" << "\t";
        } else {
            (*pftracking) << "MoMo" << "\t";
        }
    }
    bool success = nmatches > params.thresh_init;
    (*pftracking) << success << "\t";
    if(!success){
        n_frames_tracked_consecutively = 0;
    }

    std::chrono::steady_clock::time_point t_stop = std::chrono::steady_clock::now();
    std::chrono::duration<int, std::milli> t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop-t_start);


    init_pose_duration+= t_elapsed;
 //   if(n_calls % 50 == 0){
 //       std::cout << "frames tracked: " << n_calls << std::endl;
 //       std::cout << "avg init_pose_est per frame (ms): " << static_cast<float>(init_pose_duration.count())/static_cast<float>(n_calls) << std::endl;
 //   }

    return success;
}

bool TrackingStateNormal::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories){
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

    mnMatchesInliers =  track_local_map->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );

    std::chrono::steady_clock::time_point t_stop = std::chrono::steady_clock::now();
    std::chrono::duration<int, std::milli> t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop-t_start);

    refine_pose_duration+= t_elapsed;
    (*pftracking) <<  mnMatchesInliers << "\t";

    bool success = mnMatchesInliers > params.thresh_refine;
    if(success){
        n_frames_tracked_consecutively++;
    } else {
        n_frames_tracked_consecutively = 0;
    }

    if(params.reset_interval >0 && n_frames_tracked_consecutively > params.reset_interval ){ //check whether to restart tracking by artificially forcing tracking loss
        success = false;
        n_frames_tracked_consecutively = 0;
        std::cout <<"FORCING TRACKING LOSS" <<std::endl;
    }

    return success;
}

bool TrackingStateNormal::needNewKeyFrame(Frame &current_frame, Map* pMap,  unsigned int last_keyframe_id, bool force){

    // If Local Mapping is stopped for a Loop Closure do not insert keyframes
   if(!force && (thread_status->mapping.isStopped() || thread_status->mapping.isStopRequested()) )
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
    const FeatureViews views = current_frame.getViews();
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


    (*pftracking) << local_mapping_idle << "\t" << max_interval_exceeded << "\t"<< lack_close_landmarks << "\t"<< tracking_weak << "\t"<< tracking_dire <<"\t";

    bool insertKF = false;
    if(definite_insert || (optional_insert && local_mapping_idle))
    {

        // If mapping accepts keyframes (idle or queue is short), insert keyframe.
        if(local_mapping_idle || force)
        {
            insertKF = true;
        }
        else
        {
           if(thread_status->mapping.getQueueLength() < 3){
                insertKF = true;
           }
           else {
                insertKF = false;
           }
        }
    }
    (*pftracking) << insertKF << "\t";
    return insertKF;
}

    void TrackingStateNormal::clear() {
        n_frames_tracked_consecutively = 0;
    }

}