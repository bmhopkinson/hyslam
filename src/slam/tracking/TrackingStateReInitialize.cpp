//
// Created by cv-bhlab on 7/14/21.
//

#include <TrackingStateReInitialize.h>
#include <StereoInitializer.h>
#include <InitializerDataStructs.h>

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
                                                 std::map<std::string, std::unique_ptr<Trajectory>> &trajectories){
//CURRENTLY ONLY WORKS FOR STEREOCAMERA

    success = (initializer->initialize(current_frame) == 0);
    if(success){
        KeyFrame* pKFinit;
        KeyFrame* pKF2;
        std::vector<MapPoint*> mpts;
        initializer->createMap(pKFinit, pKF2, mpts);

        //TODO transform new map into global frame

        std::shared_ptr<Map> submap = pMap->createSubMap(true);
        submap->registerWithParent();//consider registered right away 
        initializer->addToMap(submap.get());

        KFnew.push_back(pKFinit);
        if(pKF2){
            KFnew.push_back(pKF2);
        }


    } else {
        *init_data = initializer->getInitializerData();
    }
    return success;
}

bool TrackingStateReInitialize::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF,
                                                   Map *pMap,
                                                   std::map<std::string, std::unique_ptr<Trajectory>> &trajectories) {
    return false;
}

void TrackingStateReInitialize::clear() {

}

bool TrackingStateReInitialize::needNewKeyFrame(Frame &current_frame, Map *pMap, unsigned int last_keyframe_id,
                                                bool force) {
    return false;
}

std::vector<KeyFrame *> TrackingStateReInitialize::createNewKeyFrame(Frame &current_frame, Map *pMap) {

}
} //END namespace