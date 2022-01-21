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
#include "FrameDrawer.h"
#include <MapPointDB.h>
#include "Map.h"
#include "ORBSLAM_datastructs.h"
#include <TrackingStateTransitionReinit.h>
#include <Tracking_datastructs.h>

#include <opencv2/core/core.hpp>

#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <unistd.h>


using namespace std;

namespace HYSLAM
{

Tracking::Tracking(FeatureVocabulary* pVoc, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer* pMapDrawer,
                   std::map<std::string, std::shared_ptr<Map> > &_maps, std::map<std::string, Camera > cam_data_,
                   const std::string &strSettingPath, MainThreadsStatus* thread_status_, FeatureFactory* factory):
            mpORBVocabulary(pVoc),
            mpFrameDrawers(pFrameDrawers), mpMapDrawer(pMapDrawer), maps(_maps) , cam_data(cam_data_), thread_status(thread_status_), feature_factory(factory)
{
    //
    std::string ftracking_name = "./run_data/tracking_data.txt";
    ftracking.open(ftracking_name);
    ftracking <<"Camera\tFrame#" << "\t" << "InitPoseMethod\t" << "InitPoseOk\t" << "mnMatchesInliers\t"<<"frame_matches_total\t"<<"nMPs_total\t" <<"nKFs_total\t"
                << "KFref_matches\t"  << "nTrackedClose\t" << "nNonTrackedClose\t"<< "local_mapping_idle\t" << "max_interval_exceeded\t"<< "lack_close_landmarks\t"<< "tracking_weak\t"<< "tracking_dire\t" <<"insertKF\t"
               << "newKFid\t" << "KF_newMps\t" <<std::endl;

    LoadSettings(strSettingPath);
    tracking_state_transition = std::make_unique<TrackingStateTransitionReinit>(config_data, cam_data, optParams, init_data, ftracking, thread_status, feature_factory);
    tracking_state_transition->setInitialState(mState, state);

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
    optParams.Info_submap_tiepoint = fSettings["Opt.Info_submap_tiepoint"];

    //  try{  //imaging camera info
    if(fSettings["Opt.Info_TrajTime"].type() != cv::FileNode::NONE ){
        optParams.Info_TrajTime = fSettings["Opt.Info_TrajTime"];
        optParams.Info_TrajTimeSE3 = fSettings["Opt.Info_TrajTimeSE3"];
        optParams.Info_ImagingTcam = fSettings["Opt.Info_ImagingTcam"];
    }

}

Tracking::~Tracking()
{
    ftracking.close();
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

    mLastProcessedState=mState[cam_cur];  //only used by viewer

    if(slam_ever_initialized) {
        UpdateLastFrame();
    }

    FrameBuffer frame_buf;
    frame_buf.push_back(mLastFrame[cam_cur]); //should probably pass pointers here
    frame_buf.push_back(mLastFrame["SLAM"]);
    mCurrentFrame.mpReferenceKF = mpReferenceKF[cam_cur];
    bool bOK = state[cam_cur]->initialPoseEstimation(mCurrentFrame, frame_buf, mpReferenceKF[cam_cur], maps[cam_cur].get(), trajectories);

    if(bOK){
        bOK = state[cam_cur]->refinePoseEstimate(mCurrentFrame, frame_buf, mpReferenceKF[cam_cur], maps[cam_cur].get(), trajectories);
    }
    mCurrentFrame.setTracked(bOK); //indicated frame was successfully tracked  - would probably be better to fold initialPoseEstimate, refinePoseEstiamte into a single method and set this value after taht combined method
    //set reference keyframe
    mCurrentFrame.mpReferenceKF = determineReferenceKeyFrame(&mCurrentFrame);

    if(mCurrentFrame.mpReferenceKF){
        mpReferenceKF[cam_cur] = mCurrentFrame.mpReferenceKF;
    }

    // If tracking was good, check if we insert a keyframe
    std::vector<KeyFrame*> newKFs;
    if(bOK)
    {
        if(cam_cur == "SLAM" && !slam_ever_initialized){
            slam_ever_initialized= true; //first time through set this when initialized (in which case bOK = true)
        }
        HandlePostTrackingSuccess();
        bool force = false;
        thread_status->mapping.setStoppable(false);
        newKFs = state[cam_cur]->newKeyFrame(mCurrentFrame, maps[cam_cur].get(), mnLastKeyFrameId, force);
        if (!newKFs.empty()) { // KeyFrame(s) created



            for(auto it = newKFs.begin(); it != newKFs.end(); ++it) {
                KeyFrame *pKFnew = *it;

                output_queue->push(pKFnew);
                std::cout << "Tracking pushed KF:  "<< pKFnew->mnId << " , from cam: " << cam_cur << std::endl;
                maps[cam_cur]->update(pKFnew);

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
    tracking_state_transition->transitionToNewState(mState, state, bOK, cam_cur);

    ftracking << std::endl;
    mCurrentFrame.propagateTracking(mLastFrame[cam_cur]);

    // Update drawer
    mpFrameDrawers[cam_cur]->Update(this);

    mLastFrame[cam_cur] = Frame(mCurrentFrame);
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe whose position may be changed by local mapping/loop closing
    KeyFrame* pKFRef = mLastFrame[cam_cur].mpReferenceKF;
    if(pKFRef && !trajectories[cam_cur]->empty()) {
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
   // SetupStates();
    thread_status->tracking.setRelease(true);

}

void Tracking::InitializeDataStructures(std::string cam_name){
  trajectories[cam_name] = std::make_shared<Trajectory>();
  mState.insert(std::make_pair(cam_name, eTrackingState::NO_IMAGES_YET) );
  init_data.insert(std::make_pair( cam_name, InitializerData() ) );
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
