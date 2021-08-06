//
// Created by cv-bhlab on 7/14/21.
//

#include <TrackingStateReInitialize.h>
#include <StereoInitializer.h>
#include <InitializerDataStructs.h>
#include <GenUtils.h>

namespace HYSLAM {
TrackingStateReInitialize::TrackingStateReInitialize(HYSLAM::optInfo optimizer_info_, HYSLAM::Camera camera_,
                                                     HYSLAM::InitializerData &init_data_,
                                                     HYSLAM::StateReInitializeParameters params_,
                                                     std::ofstream &log,
                                                     HYSLAM::MainThreadsStatus *thread_status_,
                                                     HYSLAM::FeatureFactory *factory)
                                                     : TrackingState(log, thread_status_), optimizer_info(optimizer_info_), camera(camera_), params(params_), feature_factory(factory)
{
    init_data = &init_data_;
    if (camera.sensor == 1) {  //stereo camera
        initializer = std::make_unique<StereoInitializer>(params.stereo_params );
    } else {
        std::cout << "no reinitializer available for requested camera type: "  << camera_.sensor << std::endl;
    }

}

bool TrackingStateReInitialize::initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF,
                                                      Map *pMap,
                                                      std::map<std::string, std::shared_ptr<Trajectory>> &trajectories){
//CURRENTLY ONLY WORKS FOR STEREOCAMERA

    success = (initializer->initialize(current_frame) == 0);
    if(success){
        KeyFrame* pKFinit;
        KeyFrame* pKF2;
        std::vector<MapPoint*> mpts;
        initializer->createMap(pKFinit, pKF2, mpts);

        TrajectoryElement last_tracked_frame = trajectories[camera.camName]->getLastTrackedElement();
        cv::Mat pose_last_cw = last_tracked_frame.pose();

        double t_elapsed = current_frame.mTimeStamp - last_tracked_frame.time_stamp;
        cv::Mat Vint; //estimated motion since last tracked frame
     //   std::cout << "last tracked frame id: "  << last_tracked_frame.name << " , velocity: " << last_tracked_frame.Vcw << std::endl;
     //   std::cout << "dt_vel: " <<last_tracked_frame.dt_vel << std::endl;
    //    std::cout << "t_elpased: " << t_elapsed << std::endl;
        GenUtils::ScaleVelocity(last_tracked_frame.Vcw, last_tracked_frame.dt_vel, t_elapsed, Vint);
        cv::Mat pose_current_cw = Vint * pose_last_cw;  //curent pose estimate in world to camera convention
        cv::Mat pose_inv = pose_current_cw.inv();
        initializer->transformMapSE3(pose_inv);
     //   std::cout << "Last Tracked Frame Pose: " << pose_last_cw << std::endl;
      //  std::cout << "Current Estimated Pose: " << pose_current_cw << std::endl;
     //   std::cout << "scaled velocity"  << Vint << std::endl;

      //  *(int*)0 = 0; //intentional segfault
        Tse3Parent tse3parent;
        KeyFrame* pKF_parent = last_tracked_frame.pRefKF;
        GenUtils::pauseUntilReady(pKF_parent); //this is often the newest keyframe which has not been incorporated into the map yet

        while(!pMap->setNonErasable(pKF_parent)){  //don't want this keyframe to get erased b/c it serves as reference for new submap
            pKF_parent = pKF_parent->GetParent();
        }
        tse3parent.pKFref_parent = pKF_parent;
        tse3parent.pKFref_this = pKFinit;
        cv::Mat Tse3 = pKFinit->GetPose() * pKF_parent->GetPoseInverse();
        tse3parent.Tse3 = Tse3.clone();

        std::shared_ptr<Map> submap = pMap->createSubMap(true);
        submap->setTse3Parent(tse3parent);
        submap->registerWithParent();//consider registered right away
        initializer->addToMap(submap.get());

        KFnew.push_back(pKFinit);
        if(pKF2){
            KFnew.push_back(pKF2);
        }
        std::cout << "reinitialized with new map on Frame: " << current_frame.mnId << std::endl;


    } else {
        *init_data = initializer->getInitializerData();
    }
    return success;
}

bool TrackingStateReInitialize::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF,
                                                   Map *pMap,
                                                   std::map<std::string, std::shared_ptr<Trajectory>> &trajectories) {
    return success;
}

void TrackingStateReInitialize::clear() {
    success = false;
    KFnew.clear();
    initializer = std::make_unique<StereoInitializer>(params.stereo_params );
}

bool TrackingStateReInitialize::needNewKeyFrame(Frame &current_frame, Map *pMap, unsigned int last_keyframe_id,
                                                bool force) {
    return success;
}

std::vector<KeyFrame *> TrackingStateReInitialize::createNewKeyFrame(Frame &current_frame, Map *pMap) {
    KeyFrame* pKFcur = initializer->getCurrentKF();
    current_frame =initializer->getInitializedFrame();

    current_frame.mpReferenceKF = pKFcur;
    pMap->mvpKeyFrameOrigins.push_back(pKFcur);  //used in loop closing

    return KFnew;
}
} //END namespace