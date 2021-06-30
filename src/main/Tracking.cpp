/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <Tracking.h>

#include <opencv2/core/core.hpp>

#include <Stereomatcher.h>
#include "FrameDrawer.h"
#include <MapPointDB.h>
#include "Map.h"
#include <StereoInitializer.h>
#include "ORBSLAM_datastructs.h"
#include <TrackPlaceRecognition.h>
#include <TrackingStateNormal.h>
#include <TrackingStateRelocalize.h>
#include <TrackingStateInitialize.h>
#include <TrackingStateNull.h>
#include <Tracking_datastructs.h>

#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <thread>


using namespace std;

namespace HYSLAM
{

Tracking::Tracking(System* pSys, FeatureVocabulary* pVoc, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer* pMapDrawer,
                   std::map<std::string, Map* > &_maps, std::map<std::string, Camera > cam_data_,
                   const std::string &strSettingPath, MainThreadsStatus* thread_status_, FeatureFactory* factory):
            mpORBVocabulary(pVoc), mpSystem(pSys), mpViewer(NULL),
            mpFrameDrawers(pFrameDrawers), mpMapDrawer(pMapDrawer), maps(_maps) , cam_data(cam_data_), thread_status(thread_status_), feature_factory(factory)
{
    //
    std::string ftracking_name = "./run_data/tracking_data.txt";
    ftracking.open(ftracking_name);
    ftracking <<"Camera\tFrame#" << "\t" << "InitPoseMethod\t" << "InitPoseOk\t" << "mnMatchesInliers\t"<<"frame_matches_total\t"<<"nMPs_total\t" <<"nKFs_total\t"
                << "KFref_matches\t"  << "nTrackedClose\t" << "nNonTrackedClose\t"<< "local_mapping_idle\t" << "max_interval_exceeded\t"<< "lack_close_landmarks\t"<< "tracking_weak\t"<< "tracking_dire\t" <<"insertKF\t"
               << "newKFid\t" << "KF_newMps\t" <<std::endl;

    LoadSettings(strSettingPath);
    SetupStates();

}

void Tracking::LoadSettings(std::string settings_path){
        std::cout << "LoadSettings: settings_path " << settings_path << std::endl;
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);
    config_data = cv::FileStorage(fSettings["Tracking_Config"].string(),cv::FileStorage::READ );

    cv::FileNode cameras = fSettings["Cameras"];
    for(cv::FileNodeIterator it = cameras.begin(); it != cameras.end(); it++){
        std::string cam_name  = (*it).name();
        cam_types.push_back((*it).name());
        mSensor[cam_name]  = cam_data[cam_name].sensor;
        //LoadCalibration(*it, (*it).name());
        InitializeDataStructures((*it).name());
        std::cout << "tracking loadsettings: cameras: " << cam_name << std::endl;
    }

    // load optimizer parameters
    optParams.Info_Depth = fSettings["Opt.Info_Depth"];
    optParams.Info_IMU   = fSettings["Opt.Info_IMU"];
    optParams.Info_GPS   = fSettings["Opt.Info_GPS"];
    int temp = fSettings["Opt.realtime"];
    optParams.realtime = temp; //implicit cast to bool;
    optParams.GBAinterval = fSettings["Opt.GBAinterval"];

    //  try{  //imaging camera info
    if(fSettings["Opt.Info_TrajTime"].type() != cv::FileNode::NONE ){
        optParams.Info_TrajTime = fSettings["Opt.Info_TrajTime"];
        optParams.Info_TrajTimeSE3 = fSettings["Opt.Info_TrajTimeSE3"];
        optParams.Info_ImagingTcam = fSettings["Opt.Info_ImagingTcam"];
    }

    if(mSensor["SLAM"]==System::STEREO || mSensor["SLAM"]==System::RGBD)
    {
        stereoInitFeatures = (fSettings["StereoInitFeatures"].type() == cv::FileNode::NONE) ? 500 : fSettings["StereoInitFeatures"];
        cout << endl << "Depth Threshold (Close/Far Points): " << cam_data["SLAM"].thDepth << endl;
        cout << "Stereo Init Features: " << stereoInitFeatures << endl;
    }

}

Tracking::~Tracking()
{
    ftracking.close();
}
/*
void Tracking::SetLocalMapper(Mapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}
*/
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::Run(){

    while(true) {
        if(inputAvailable()){
            ImageFeatureData image_feature_data = input_queue->pop();
            track(image_feature_data);
        } else if (thread_status->tracking.isFinishRequested()){
            break;
        }

        if(Stop())
        {
            std::cout << "stopping tracking" << std::endl;
            // Safe area to stop
            while(!thread_status->tracking.isRelease())
            {
                usleep(3000);
            }
            //clear stop related flags and resume operation
            thread_status->tracking.clearPostStop();
            std::cout << "restarting tracking" << std::endl;
        }


        std::this_thread::sleep_for(std::chrono::microseconds(1000) );
    }
    thread_status->tracking.setIsFinished(true);
    std::cout << "tracking Finished" << std::endl;
}

bool Tracking::inputAvailable(){
    return (input_queue->size() > 0);
}
//cv::Mat Tracking::track(FeatureViews LMviews, cv::Mat &image, std::string cam_name, const Imgdata &img_data, const SensorData &sensor_data){
cv::Mat Tracking::track(ImageFeatureData &track_data){
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    cam_cur = track_data.img_data.camera;
    mImGray = track_data.image;
    Camera cam_data_cur = cam_data[cam_cur];

    bool is_stereo = false;
    if(cam_data_cur.sensor == 0){
        is_stereo = false;
    } else if( cam_data_cur.sensor == 1 ){
        is_stereo = true;
    }

    mCurrentFrame = Frame( track_data.img_data.time_stamp, track_data.LMviews, mpORBVocabulary,cam_data[cam_cur], track_data.img_data.name, track_data.sensor_data, is_stereo);

    _Track_();

    std::chrono::steady_clock::time_point t_stop = std::chrono::steady_clock::now();
    std::chrono::duration<int, std::milli> t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop-t_start);
   // std::cout << "tracking duration (ms):  " << t_elapsed.count() << std::endl;

    return mCurrentFrame.mTcw.clone();
}

void Tracking::_Track_()
{
    ftracking << cam_cur<< "\t" << mCurrentFrame.mnId << "\t";

    // ADDITION: Tracking state monitoring
    nPoints = 0;
    nObserved = 0;
    mLastProcessedState=mState[cam_cur];  //only used by viewer

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(maps[cam_cur]->mMutexMapUpdate);

    if(slam_ever_initialized) {
        UpdateLastFrame();
    }

    FrameBuffer frame_buf;
    frame_buf.push_back(mLastFrame[cam_cur]); //should probably pass pointers here
    frame_buf.push_back(mLastFrame["SLAM"]);
    mCurrentFrame.mpReferenceKF = mpReferenceKF[cam_cur];
    bool bOK = state[cam_cur]->initialPoseEstimation(mCurrentFrame, frame_buf, mpReferenceKF[cam_cur], maps[cam_cur], trajectories);

    if(bOK){
        bOK = state[cam_cur]->refinePoseEstimate(mCurrentFrame, frame_buf, mpReferenceKF[cam_cur], maps[cam_cur], trajectories);
    }
    mCurrentFrame.setTracked(bOK); //indicated frame was successfully tracked  - would probably be better to fold initialPoseEstimate, refinePoseEstiamte into a single method and set this value after taht combined method
    //set reference keyframe
    mCurrentFrame.mpReferenceKF = determineReferenceKeyFrame(&mCurrentFrame);

    if(mCurrentFrame.mpReferenceKF){
        // std::cout << "determineReferenceKeyFrame: frame: " << mCurrentFrame.mnId << " ,refKF: " <<  mCurrentFrame.mpReferenceKF->mnId << std::endl;
        mpReferenceKF[cam_cur] = mCurrentFrame.mpReferenceKF;
    }



    // If tracking was good, check if we insert a keyframe
    std::vector<KeyFrame*> newKFs;
    if(bOK)
    {
        HandlePostTrackingSuccess();
        bool force = false;
        if( recent_init[cam_cur] > 0 ){
            force = true;
        //    std::cout << "forcing new keyframe due to recent initialization for this and more frames: " <<  recent_init[cam_cur] << std::endl;
        }

        thread_status->mapping.setStoppable(false);
        newKFs = state[cam_cur]->newKeyFrame(mCurrentFrame, maps[cam_cur], mnLastKeyFrameId, force);
        if (!newKFs.empty()) { // KeyFrame(s) created
            for(auto it = newKFs.begin(); it != newKFs.end(); ++it) {
                KeyFrame *pKFnew = *it;
                output_queue->push(pKFnew);
                std::cout << "Tracking pushed KF:  "<< pKFnew->mnId << " , from cam: " << cam_cur << std::endl;
                maps[cam_cur]->getKeyFrameDB()->update(pKFnew);

                if( recent_init[cam_cur] > 0 ){
                    recent_init[cam_cur]--;
                }
            }

            mpReferenceKF[cam_cur] = newKFs.back();
            mnLastKeyFrameId = mCurrentFrame.mnId;
        }
        thread_status->mapping.setStoppable(true);


        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        LandMarkMatches matches = mCurrentFrame.getLandMarkMatches();
        for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
            int LMid = it->first;
            if(mCurrentFrame.isOutlier(LMid)){
                mCurrentFrame.removeLandMarkAssociation(LMid);
            }
        }

    }//end keyframe insertion and clean up

    if(!mCurrentFrame.mpReferenceKF)  //WHY DOES THE REFERENCE KEYFRAME KEEP GETTING UPDATED!!!!
        mCurrentFrame.mpReferenceKF = mpReferenceKF[cam_cur];

    //append current frame's data to trajectory
    if(slam_ever_initialized) {
        trajectories[cam_cur]->push_back(mCurrentFrame);
    }

    //set next state
    eTrackingState next_state;
    TrackingState* pnext_track_state;
    if(mState[cam_cur] == eTrackingState::INITIALIZATION){
        if(bOK){

            HandlePostInit(newKFs.back(), maps[cam_cur],  cam_cur);
            next_state = eTrackingState::NORMAL;
            delete state[cam_cur];
            pnext_track_state = state_options[cam_cur]["NORMAL"];
        } else {
            next_state =  eTrackingState::INITIALIZATION;
            pnext_track_state = state[cam_cur];
        }
    }
    else if(mState[cam_cur] == eTrackingState::NORMAL) {
        if (bOK) {
            pnext_track_state = state_options[cam_cur]["NORMAL"];
            next_state = eTrackingState::NORMAL;
        } else {
            if (cam_cur == "SLAM") {
                pnext_track_state = state_options[cam_cur]["RELOCALIZE"];
                next_state = eTrackingState::RELOCALIZATION;
                std::cout << "SLAM CAMERA LOST TRACKING, TRYING TO RELOCALIZE" << std::endl;

                if(state.find("Imaging") != state.end()){ //if there's an imaging camera set it to NULL state b/c we can't track it if SLAM tracking is lost
                    state["Imaging"]  = state_options["Imaging"]["NULL"];
                    mState["Imaging"] = eTrackingState::NULL_STATE;
                }

            } else {
                //  state[cam_cur] = state_options["INITIALIZE"]; //don't try to relocalize accessory cameras - just reinitialize
                next_state = eTrackingState::INITIALIZATION;
                cv::FileNode cam_states = config_data["Cameras"];
                cv::FileNode state_config = config_data["States"];
                StateInitializeParameters state_initialize_params(state_config[cam_states[cam_cur]["Initialize"].string()], config_data["Strategies"]);
                pnext_track_state = new TrackingStateInitialize(optParams, cam_data[cam_cur], init_data[cam_cur],
                                                                state_initialize_params, ftracking, thread_status, feature_factory);
            }
        }
    }
    else if(mState[cam_cur] == eTrackingState::RELOCALIZATION){
        if(bOK){
            pnext_track_state = state_options[cam_cur]["NORMAL"];
            next_state = eTrackingState::NORMAL;

            if(cam_cur == "SLAM" && state.find("Imaging") != state.end()){ //if the SLAM camera was localized and there's an imaging camera, can allow imaging to start operating again
                mState["Imaging"] = eTrackingState::INITIALIZATION;
                cv::FileNode cam_states = config_data["Cameras"];
                cv::FileNode state_config = config_data["States"];
                StateInitializeParameters state_initialize_params(state_config[cam_states["Imaging"]["Initialize"].string()], config_data["Strategies"]);
                pnext_track_state = new TrackingStateInitialize(optParams, cam_data["Imaging"], init_data["Imaging"],
                                                                state_initialize_params, ftracking, thread_status, feature_factory);
            }
        }
    }
    mState[cam_cur] = next_state;
    state[cam_cur] = pnext_track_state;

    ftracking << std::endl;
   // mCurrentFrame.validateMatches();
   // mLastFrame[cam_cur].validateMatches();
    mCurrentFrame.propagateTracking(mLastFrame[cam_cur]);

    // Update drawer
    mpFrameDrawers[cam_cur]->Update(this);

    mLastFrame[cam_cur] = Frame(mCurrentFrame);
}

void Tracking::SetupStates(){
  //  std::map<std::string, TrackingState*> state_options;
 // cv::FileStorage config_data(config_file, cv::FileStorage::READ);
 cv::FileNode cam_states = config_data["Cameras"];
 cv::FileNode state_config = config_data["States"];
 cv::FileNode strategy_config = config_data["Strategies"];

  for(auto it = cam_data.begin(); it != cam_data.end(); ++it) {
      std::string cam_name = it->first;
      Camera cam = it->second;
      //Initialization
      StateInitializeParameters state_initialize_params(state_config[cam_states[cam_name]["Initialize"].string()],
                                                        strategy_config);
      state[cam_name] = new TrackingStateInitialize(optParams, cam, init_data[cam.camName], state_initialize_params,
                                                       ftracking, thread_status, feature_factory);
      mState[cam_name] = eTrackingState::INITIALIZATION;

      //Normal
      StateNormalParameters state_normal_params(state_config[cam_states[cam_name]["Normal"].string()], strategy_config);
      state_options[cam_name]["NORMAL"] = new TrackingStateNormal(optParams, state_normal_params,
                                                                  ftracking, thread_status, feature_factory);

      //Relocalize
      StateRelocalizeParameters state_relocalize_params(state_config[cam_states[cam_name]["Relocalize"].string()],
                                                        strategy_config);
      state_options[cam_name]["RELOCALIZE"] = new TrackingStateRelocalize(optParams, state_relocalize_params,
                                                                          ftracking, thread_status, feature_factory);

      //Null
      state_options[cam_name]["NULL"] = new TrackingStateNull(ftracking, thread_status);
  }
}

int Tracking::HandlePostInit(KeyFrame* pKFcurrent,Map* pMap,std::string cam_name ){
    if(cam_name == "SLAM"){
        slam_ever_initialized = true;
        mvpLocalMapPoints=pMap->GetAllMapPoints();
        std::cout << "initialized with " << mvpLocalMapPoints.size() << " new mpts" << std::endl;
        pMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMapDrawer->SetCurrentCameraPose(pKFcurrent->GetPose());
    }

    recent_init[cam_name] = 5; // will rapidly insert 5 more keyframes
    std::cout << "finished postinitialization: on KF " << pKFcurrent->mnId <<std::endl;
    return 0;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe whose position may be changed by local mapping/loop closing
    KeyFrame* pKFRef = mLastFrame[cam_cur].mpReferenceKF;
    if(pKFRef && !trajectories[cam_cur]->empty()) {
  //      std::cout << "in UpdateLast frame, refKF is: " << pKFRef->mnId << std::endl;
        TrajectoryElement te_back = trajectories[cam_cur]->back();
        cv::Mat Tlr = te_back.Tcr;
        mLastFrame[cam_cur].SetPose(Tlr * pKFRef->GetPose());
    }


    //update any replaced mappoints
    const LandMarkMatches matches = mLastFrame[cam_cur].getLandMarkMatches();
    for(auto it = matches.cbegin(); it != matches.cend(); ++it){
        int LMid = it->first;
        MapPoint* pMP = it->second;
        if(!pMP){continue;}
        MapPoint* pRep = pMP->GetReplaced();
        if(pRep){
            mLastFrame[cam_cur].associateLandMark(LMid, pRep, true);
        }
    }
}

KeyFrame* Tracking::determineReferenceKeyFrame(Frame* pcurrent_frame){
    // Each map point vote for the keyframes in which it has been observed
    KeyFrame * pKFbest_ref = nullptr;
    map<KeyFrame*,int> keyframeCounter;
    const LandMarkMatches matches = pcurrent_frame->getLandMarkMatches();
    for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
        int LMid = it->first;
        MapPoint *pMP = it->second;
        if(!pMP){continue;}
        if(!pMP->isBad())
        {
            const map<KeyFrame*,size_t> observations = pMP->GetObservations();
            for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                keyframeCounter[it->first]++;
        }
        else
        {
            pcurrent_frame->removeLandMarkAssociation(LMid);
        }
    }

    if(keyframeCounter.empty())
        return  pKFbest_ref;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }
    }

    pKFbest_ref = pKFmax;
    return pKFbest_ref;
}

bool Tracking::Stop()
{
    if(thread_status->tracking.isStopRequested() && thread_status->tracking.isStoppable())
    {
        thread_status->tracking.setIsStopped(true);
        return true;
        std::cout << "stopping tracking" << std::endl;
    }
    return false;

}

void Tracking::Reset()
{

    thread_status->tracking.setStopRequested(true);
    // Wait until Tracking has effectively stopped
    while(!thread_status->tracking.isStopped())
    {
        thread_status->tracking.setStopRequested(true); //someone else may have cleared the initial request - so keep putting it in
        usleep(1000);
    }

    while(input_queue->size() > 0 ){
        input_queue->pop();
    }
    thread_status->tracking.setQueueLength(input_queue->size());

    for(std::vector<std::string>::iterator vit = cam_types.begin(); vit != cam_types.end(); vit++){
        std::string this_cam = *vit;
        trajectories[this_cam]->clear();
        slam_ever_initialized = false;
        mCurrentFrame = Frame();
        mLastFrame[this_cam]  = Frame();
        mpFrameDrawers[this_cam]->clear();

    }
    SetupStates();
    thread_status->tracking.setRelease(true);

}

void Tracking::InitializeDataStructures(std::string cam_name){
  //  Trajectory trajectory;
  //trajectories.insert(std::make_pair( cam_name, trajectory) );
  trajectories[cam_name] = std::make_unique<Trajectory>();
  mState.insert(std::make_pair(cam_name, eTrackingState::NO_IMAGES_YET) );
  init_data.insert(std::make_pair( cam_name, InitializerData() ) );
  recent_init.insert(std::make_pair(cam_name, 0 ) );
  mLastFrame.insert(std::make_pair(cam_name, Frame() ) );

}

void Tracking::HandlePostTrackingSuccess(){
  // Update motion model - only for SLAM
  if(cam_cur == "SLAM"){
    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
  }

  // Clean VO matches - should be able to remove this
  LandMarkMatches matches = mCurrentFrame.getLandMarkMatches();
  for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
      int LMid = it->first;
      MapPoint* pMP = it->second;
      if(!pMP){continue;}
      if(pMP->Observations()<1)
      {
          mCurrentFrame.setOutlier(LMid, false) ;
          mCurrentFrame.removeLandMarkAssociation(LMid);
      }
  }


  int n_valid_matches = mCurrentFrame.getLandMarkMatches().numValidMatches();
  ftracking <<  n_valid_matches << "\t" << maps[cam_cur]->MapPointsInMap() << "\t" <<   maps[cam_cur]->KeyFramesInMap() <<"\t";

}

} //namespace ORB_SLAM
