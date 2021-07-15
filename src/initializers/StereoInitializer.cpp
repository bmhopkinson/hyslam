#include <StereoInitializer.h>
#include <FeatureViews.h>
#include <MapPoint.h>

namespace HYSLAM{

StereoInitializer::StereoInitializer(StereoInitializerParameters params_) :  params(params_)
     {}

int StereoInitializer::initialize(Frame &frame){
 if(frame.N > params.N_min_features)
 {
     frame_init = frame;
     init_success = true;
     return 0;
 } else {
     return -1;
 }
}

int StereoInitializer::createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints){
    // Set Frame pose to the origin
    frame_init.SetPose(cv::Mat::eye(4,4,CV_32F));

    // Create KeyFrame
    pKFinit = new KeyFrame(frame_init);
    pKFinit->SetRefQuat(frame_init.getSensorData());
    pKFinit->SetRefGPS(frame_init.getSensorData());

    // Create MapPoints and associate to Frame/KeyFrame
    const FeatureViews views = frame_init.getViews();
    for(int i=0; i<frame_init.N;i++)
    {
        float z = views.depth(i);
        if(z>0)
        {
            cv::Mat world_pos = frame_init.UnprojectStereo(i);
            MapPoint* pMP = new MapPoint(world_pos);
            mpts.push_back(pMP);
            int res = frame_init.associateLandMark(i, pMP, true);
            pKFinit->associateLandMark(i, pMP, true);
            pKFinit->setOutlier(i,false);
            mpts_to_keypts[pMP] = i;
        }
    }

    pKF1 = pKFinit;
    pKF2 = nullptr;
    mappoints = mpts;
    map_created = true;

    return 0;

}

int StereoInitializer::transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T){
    return -1; //not implemented yet
}

Frame StereoInitializer::getInitializedFrame() {
    if(!init_success || !map_created){
        std::cout << "WARNING: StereoInitialization not Finalized, Frame not Complete!!" << std::endl;
    }
    return frame_init;

}

KeyFrame* StereoInitializer::getCurrentKF(){
    return pKFinit;
}

int StereoInitializer::addToMap(Map* pMap){
   // pKFinit->setMap(pMap);
    pMap->AddKeyFrame(pKFinit);
   // std::cout << "adding to Map KeyFrame: " << pKFinit->mnId <<std::endl;

    //mappoint
    for(auto it = mpts.begin(); it != mpts.end(); ++it){
        MapPoint* pMP = *it;
        pMap->AddMapPoint(pMP, pKFinit, mpts_to_keypts[pMP]);
        pMP->SetProtection(params.new_mpt_protection, pKFinit->mnId); //protect from culling for 5 new keyframes
    }

    return 0;
}

void StereoInitializer::clear(){
    init_success = false;
    map_created = false;

    frame_init = Frame();
    pKFinit = nullptr;
    mpts.clear();
    scale = 1.0000; // map scale
    mpts_to_keypts.clear();

    Initializer::clear();

}

int StereoInitializer::transformMapSE3(cv::Mat &Twc_SE3) {
    cv::Mat KF_pose_cw = pKFinit->GetPose(); //in world to camera convention
    KF_pose_cw =  KF_pose_cw * Twc_SE3.inv();
    pKFinit->SetPose(KF_pose_cw);
    frame_init.SetPose(KF_pose_cw);

    for(auto it = mpts.begin(); it != mpts.end(); ++it){
        MapPoint* pMP = *it;
        pMP->applyTransform(Twc_SE3);
    }
    return 0;
}


} // end namespace
