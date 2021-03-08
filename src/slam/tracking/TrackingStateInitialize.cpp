#include <TrackingStateInitialize.h>
#include <MonoInitializer.h>
#include <StereoInitializer.h>
#include <InitializerDataStructs.h>
#include <GlobalBundleAdjustment.h>

namespace HYSLAM {
TrackingStateInitialize::TrackingStateInitialize(optInfo optimizer_info_, Camera camera_, InitializerData &init_data_,  StateInitializeParameters params_, std::ofstream &log, MainThreadsStatus* thread_status_) :
    TrackingState(log, thread_status_), optimizer_info(optimizer_info_), camera(camera_), params(params_)
{
 //   cv::FileNode config_data_strategies =config_data["Strategies"];
 //   cv::FileNode config_init = config_data_strategies["Initialize"];
    init_data = &init_data_;
    if (camera.sensor == 1) {  //stereo camera
       // StereoInitializerParameters siparams(config_init["Stereo"]);
        initializer = std::make_unique<StereoInitializer>(params.stereo_params );
    } else if (camera.sensor == 0){ //mono camera
      //  MonoInitializerParameters miparams(config_init["Mono"]);
        initializer = std::make_unique<MonoInitializer>(params.mono_params );
    } else {
        std::cout << "no initializer available for requested camera type: "  << camera_.sensor << std::endl;
    }
}

bool TrackingStateInitialize::initialPoseEstimation( Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map< std::string, std::unique_ptr<Trajectory> > &trajectories){
    const Frame& last_frame = frames[0];
    const Frame& last_slamframe = frames[1];

    success = (initializer->initialize(current_frame) == 0);
    if(success){
        KeyFrame* pKFinit;
        KeyFrame* pKF2;
        std::vector<MapPoint*> mpts;
        initializer->createMap(pKFinit, pKF2, mpts);

        if((camera.sensor == 0) && camera.camName != "SLAM"){
            initializer->transformMap(trajectories.at("SLAM").get(), last_slamframe, camera.Tcam);
        }
        initializer->addToMap(pMap);

        if((camera.sensor == 0)  && camera.camName == "SLAM") {
            HandlePostMonoInitSLAM(pKFinit, pKF2, pMap,trajectories.at("SLAM").get() );
        }

        KFnew.push_back(pKFinit);
        if(pKF2){
            KFnew.push_back(pKF2);
        }


    } else {
        *init_data = initializer->getInitializerData();
    }
    return success;

}

//no-op
bool TrackingStateInitialize::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map< std::string, std::unique_ptr<Trajectory> > &trajectories){
    return success;
}

void TrackingStateInitialize::clear(){
    success = false;
    KFnew.clear();
    initializer->clear();
}

bool TrackingStateInitialize::needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force){
    return success;
}

std::vector<KeyFrame*> TrackingStateInitialize::createNewKeyFrame(Frame &current_frame, Map* pMap){
    KeyFrame* pKFcur = initializer->getCurrentKF();
    current_frame =initializer->getInitializedFrame();
/*
    for(std::vector<KeyFrame*>::iterator vit = KFnew.begin(); vit != KFnew.end(); ++vit){
        KeyFrame* pKF = *vit;
       // pLocalMapper->InsertKeyFrame(pKF);
       // pMap->getKeyFrameDB()->update(pKF);
    }
*/
    current_frame.SetPose(pKFcur->GetPose());  //think I should be able to move this up to initialPoseEstimation which seems more appropriate
    current_frame.mpReferenceKF = pKFcur;
    pMap->mvpKeyFrameOrigins.push_back(pKFcur);  //used in loop closing


    return KFnew;
}

int TrackingStateInitialize::HandlePostMonoInitSLAM(KeyFrame* pKFini, KeyFrame* pKFcur, Map* pMap, Trajectory* trajectory){
    std::cout << "New Map created with " << pMap->MapPointsInMap() << " points" << std::endl;
    optimizer_info.GBAtype = 1; //routine GBA
    std::vector<KeyFrame*> vpKFfixed;  //fix KeyFrame 0
    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    for(std::vector<KeyFrame*>::iterator it  = vpKFs.begin(); it != vpKFs.end() ; it++){
        KeyFrame* pKF = *it;
        if(pKF->mnId == 0){
            vpKFfixed.push_back(pKF);
        }
    }

    int nIter = 20;
    bool  bRobust = false;
    bool mbStopGBA = false;
    g2o::Trajectory traj_g2o = trajectory->convertToG2O(); //trajectory will be empty. this should be ok b/c there won't be any imaging camera data yet either
    GlobalBundleAdjustment globalBA(vpKFfixed,  0, nIter ,  bRobust, &mbStopGBA, pMap, traj_g2o, optimizer_info);
    globalBA.Run();

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        std::cout << "Wrong initialization, should reset!!!!..." << std::endl;
      //  Reset();
        return -1;
    }

    // Scale initial baseline -
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    std::vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    return 0;
}

} //end namespace