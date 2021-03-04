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

#include <ORBstereomatcher.h>
#include <ORBFinder.h>
#include <ORBUtil.h>
#include "FrameDrawer.h"
#include <MapPointDB.h>
#include "Map.h"
#include <StereoInitializer.h>
#include "ImagingBundleAdjustment.h"
#include "ORBSLAM_datastructs.h"
#include <TrackPlaceRecognition.h>
#include <TrackingStateNormal.h>
#include <TrackingStateRelocalize.h>
#include <TrackingStateInitialize.h>
#include <Tracking_datastructs.h>
#include "g2o/types/sba/Trajectory_g2o.h"

#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <thread>


using namespace std;

namespace HYSLAM
{

    Tracking::Tracking(System* pSys, ORBVocabulary* pVoc, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer* pMapDrawer,
               std::map<std::string, Map* > &_maps, std::map<std::string, Camera > cam_data_, const std::string &strSettingPath):
     mbVO(false), mpORBVocabulary(pVoc), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawers(pFrameDrawers), mpMapDrawer(pMapDrawer), maps(_maps) , cam_data(cam_data_), mnLastRelocFrameId(0)

{
    //
    std::string ftracking_name = "./run_data/tracking_data.txt";
    ftracking.open(ftracking_name);
    ftracking <<"Camera\tFrame#" << "\t" << "InitPoseMethod\t" << "InitPoseOk\t" << "mnMatchesInliers\t"<<"frame_matches_total\t"<<"nMPs_total\t" <<"nKFs_total\t"
                << "KFref_matches\t"  << "nTrackedClose\t" << "nNonTrackedClose\t" <<"insertKF\t"
                << "c1a\tc1b\tc1c\tc2\t"<< "newKFid\t" << "KF_newMps\t" <<std::endl;

    // Load camera parameters from settings file
    ORBextractorSettings ORBextractor_settings;
    std::string tracking_config_file;
    LoadSettings(strSettingPath, ORBextractor_settings, tracking_config_file);

    mpORBextractorLeft = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true) ,ORBextractor_settings);

    if(mSensor["SLAM"]==System::STEREO)
        mpORBextractorRight = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true) , ORBextractor_settings);

   // if(mSensor["SLAM"]==System::MONOCULAR)
   ORBextractorSettings ORBextractor_settings_init;
   ORBextractor_settings_init = ORBextractor_settings;
   ORBextractor_settings_init.nFeatures = 3*ORBextractor_settings.nFeatures;
   mpIniORBextractor = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true) , ORBextractor_settings_init);

   SetupStates();

}

void Tracking::LoadSettings(std::string settings_path, ORBextractorSettings &ORBext_settings, std::string &tracking_filename){
        std::cout << "LoadSettings: settings_path " << settings_path << std::endl;
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);
    //tracking_filename = fSettings["Tracking_Config"].string();
    config_data = cv::FileStorage(fSettings["Tracking_Config"].string(),cv::FileStorage::READ );
   // std::cout << "tracking_filename: " << tracking_filename << std::endl;

    cv::FileNode cameras = fSettings["Cameras"];
    for(cv::FileNodeIterator it = cameras.begin(); it != cameras.end(); it++){
        std::string cam_name  = (*it).name();
        cam_types.push_back((*it).name());
        mSensor[cam_name]  = cam_data[cam_name].sensor;
        //LoadCalibration(*it, (*it).name());
        InitializeDataStructures((*it).name());
        std::cout << "tracking loadsettings: cameras: " << cam_name << std::endl;
    }

    // Load ORB parameters

    ORBext_settings.nFeatures = fSettings["ORBextractor.nFeatures"];
    ORBext_settings.fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    ORBext_settings.nLevels = fSettings["ORBextractor.nLevels"];
    ORBext_settings.fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    ORBext_settings.fMinThFAST = fSettings["ORBextractor.minThFAST"];
    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << ORBext_settings.nFeatures << std::endl;
    std::cout << "- Scale Levels: " << ORBext_settings.nLevels << std::endl;
    std::cout << "- Scale Factor: " << ORBext_settings.fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << ORBext_settings.fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << ORBext_settings.fMinThFAST << std::endl;

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
/*
    if(mSensor["SLAM"]==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }
*/
}

Tracking::~Tracking()
{
    ftracking.close();
}

void Tracking::SetLocalMapper(Mapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data)
{
    cam_cur = img_data.camera;
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    mImGray     = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);
    imGrayRight = PreProcessImg(imGrayRight, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    Camera cam_data_cur = cam_data[cam_cur];
    cam_data_cur.mnMaxX = mImGray.cols;
    cam_data_cur.mnMaxY = mImGray.rows;

    // ORB extraction - all this needs to be moved to separate class - perhaps even just a parallel LandMark Extraction thread
    // this has nothing to do w/ Tracking - frame should be created in LandMark Extraction class and then passed into Tracking to track frame
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    cv::Mat mDescriptors, mDescriptorsRight;
    std::thread orb_thread(ORBUtil::extractORB, mpORBextractorLeft, std::ref(mImGray), std::ref(mvKeys),std::ref( mDescriptors) ); 
 //   (*mpORBextractorLeft)(mImGray      , cv::Mat(), mvKeys     ,  mDescriptors );
    (*mpORBextractorRight)(imGrayRight, cv::Mat(), mvKeysRight,  mDescriptorsRight );
    orb_thread.join();
    ORBExtractorParams orb_params(mpORBextractorLeft);
    ORBViews LMviews(mvKeys, mvKeysRight, mDescriptors, mDescriptorsRight, orb_params);
    ORBstereomatcher stereomatch(mpORBextractorLeft, mpORBextractorRight, LMviews, cam_data_cur);
    stereomatch.computeStereoMatches();
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    stereomatch.getData(LMviews);

    mCurrentFrame = Frame( img_data.time_stamp, LMviews, mpORBVocabulary, cam_data_cur  , img_data.name, sensor_data, true );
    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data)
{
    cam_cur = img_data.camera;
    mImGray = im;
    mImGray = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    Camera cam_data_cur = cam_data[cam_cur];
    cam_data_cur.mnMaxX = mImGray.cols;
    cam_data_cur.mnMaxY = mImGray.rows;

    // ORB extraction
    FeatureExtractor* extractor;
    if(mState[cam_cur]==eTrackingState::INITIALIZATION || mState[cam_cur]==eTrackingState::NO_IMAGES_YET){
        extractor = mpIniORBextractor;
    }
    else {
        extractor = mpORBextractorLeft;
    }
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    (*extractor)(mImGray      , cv::Mat(), mvKeys     ,  mDescriptors );
    ORBExtractorParams orb_params(extractor);
    ORBViews LMviews(mvKeys, mDescriptors,  orb_params);

    mCurrentFrame = Frame( img_data.time_stamp, LMviews, mpORBVocabulary,cam_data_cur, img_data.name, sensor_data, false);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::trackMono(ORBViews LMviews, cv::Mat &image, std::string cam_name, const Imgdata &img_data, const SensorData &sensor_data){
    cam_cur = img_data.camera;
    mImGray = image;
    Camera cam_data_cur = cam_data[cam_cur];

    mCurrentFrame = Frame( img_data.time_stamp, LMviews, mpORBVocabulary,cam_data[cam_cur], img_data.name, sensor_data, false);

    Track();
    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::trackStereo(ORBViews LMviews, cv::Mat &image, std::string cam_name, const Imgdata &img_data, const SensorData &sensor_data){
    cam_cur = img_data.camera;
    mImGray = image;
    Camera cam_data_cur = cam_data[cam_cur];
    mCurrentFrame = Frame( img_data.time_stamp, LMviews, mpORBVocabulary, cam_data_cur  , img_data.name, sensor_data, true );

    Track();
    return mCurrentFrame.mTcw.clone();
}



void Tracking::Track()
{
    ftracking << cam_cur<< "\t" << mCurrentFrame.mnId << "\t";
    //stop if needed - non-realtime only
    if(Stop())
    {
    // Safe area to stop
       while(isStopped())
       {
           usleep(3000);
       }
   }

    // ADDITION: Tracking state monitoring
    nPoints = 0;
    nObserved = 0;
    mLastProcessedState=mState[cam_cur];  //only used by viewer

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(maps[cam_cur]->mMutexMapUpdate);

    bOK = false;
    UpdateLastFrame();

    FrameBuffer frame_buf;
    frame_buf.push_back(mLastFrame[cam_cur]); //should probably pass pointers here
    frame_buf.push_back(mLastFrame["SLAM"]);
    mCurrentFrame.mpReferenceKF = mpReferenceKF[cam_cur];
    bOK = state[cam_cur]->initialPoseEstimation(mCurrentFrame, frame_buf, mpReferenceKF[cam_cur], maps[cam_cur], trajectories);

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
    KeyFrame* pKFnew = nullptr;
    if(bOK)
    {
        HandlePostTrackingSuccess();
        bool force = false;
        if( recent_init[cam_cur] > 0 ){ //refine this - recent_mono_init should be map for each cam
            force = true;
            recent_init[cam_cur]--;
        }
        pKFnew = state[cam_cur]->newKeyFrame(mCurrentFrame, maps[cam_cur], mpLocalMapper, mnLastKeyFrameId, force);
        if( pKFnew){ //this logic needs to go somewhere
            mpReferenceKF[cam_cur] =  pKFnew;
            mnLastKeyFrameId = mCurrentFrame.mnId;
       //     mpLastKeyFrame = pKFnew;
        }

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


    // Reset if the camera get lost soon after initialization
    if(mState[cam_cur]==eTrackingState::RELOCALIZATION)
    {
        if(maps[cam_cur]->KeyFramesInMap()<=5)
        {
            std::cout << "Track lost soon after initialisation, reseting..." << std::endl;
            mpSystem->Reset();
            return;
        }
    }

    //append current frame's data to trajectory
    trajectories[cam_cur]->push_back(mCurrentFrame);

    //set next state
    eTrackingState next_state;
    TrackingState* pnext_track_state;
    if(mState[cam_cur] == eTrackingState::INITIALIZATION){
        if(bOK){

            HandlePostInit(pKFnew, maps[cam_cur],  cam_cur);
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
            } else {
                //  state[cam_cur] = state_options["INITIALIZE"]; //don't try to relocalize accessory cameras - just reinitialize
                next_state = eTrackingState::INITIALIZATION;
                cv::FileNode cam_states = config_data["Cameras"];
                cv::FileNode state_config = config_data["States"];
                StateInitializeParameters state_initialize_params(state_config[cam_states[cam_cur]["Initialize"].string()], config_data["Strategies"]);
                pnext_track_state = new TrackingStateInitialize(optParams, cam_data[cam_cur], init_data[cam_cur],
                                                                state_initialize_params, ftracking);
            }
        }
    }
    else if(mState[cam_cur] == eTrackingState::RELOCALIZATION){
        if(bOK){
            pnext_track_state = state_options[cam_cur]["NORMAL"];
            next_state = eTrackingState::NORMAL;
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
                                                       ftracking);
      mState[cam_name] = eTrackingState::INITIALIZATION;

      //Normal
      StateNormalParameters state_normal_params(state_config[cam_states[cam_name]["Normal"].string()], strategy_config);
      state_options[cam_name]["NORMAL"] = new TrackingStateNormal(optParams, state_normal_params, ftracking);

      //Relocalize
      StateRelocalizeParameters state_relocalize_params(state_config[cam_states[cam_name]["Relocalize"].string()],
                                                        strategy_config);
      state_options[cam_name]["RELOCALIZE"] = new TrackingStateRelocalize(optParams, state_relocalize_params, ftracking);
  }
}

int Tracking::HandlePostInit(KeyFrame* pKFcurrent,Map* pMap,std::string cam_name ){
    if(cam_name == "SLAM"){
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

void Tracking::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;

}

bool Tracking::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested)
    {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void Tracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;

    cout << "Tracking RELEASE" << endl;
}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;


    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;

    for(std::vector<std::string>::iterator vit = cam_types.begin(); vit != cam_types.end(); vit++){
        std::string this_cam = *vit;
        maps[this_cam]->clear(); // Clear Map (this erase MapPoints and KeyFrames)

        mState[this_cam] = eTrackingState::NO_IMAGES_YET; //need to clear from all cams
        trajectories[this_cam]->clear();

      /*  if(mpInitializer[this_cam])
        {
            delete mpInitializer[this_cam];
            mpInitializer[this_cam] = static_cast<Initializer*>(NULL);
        }
        */
    }

    if(mpViewer)
        mpViewer->Release();
}


void Tracking::LoadCalibration(const cv::FileNode &camera, std::string cam_name)
{
    Camera cam_info;
    cam_info.loadData(camera);
    mSensor[cam_name]  = cam_info.sensor;
    cam_data.insert( std::make_pair(cam_name, cam_info) );

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
/*
void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}
*/

cv::Mat Tracking::PreProcessImg(cv::Mat &img, bool mbRGB, float fscale){

    cv::resize(img, img, cv::Size(), fscale, fscale);

    if(img.channels()==3)
    {
        if(mbRGB)
            cvtColor(img, img ,CV_RGB2GRAY);
        else
            cvtColor(img, img ,CV_BGR2GRAY);
    }
    else if(img.channels()==4)
    {
        if(mbRGB)
            cvtColor(img, img ,CV_RGBA2GRAY);
        else
            cvtColor(img, img ,CV_BGRA2GRAY);
    }

    return img;
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

void Tracking::RunImagingBundleAdjustment(){
  //stop LocalMapping and LoopClosing
  mpLocalMapper->RequestStop();

  // Wait until Local Mapping has effectively stopped
  while(!mpLocalMapper->isStopped())
  {
      usleep(1000);
  }
  while(mpLoopClosing->isRunningGBA()){  //can't stop loop closing right now - add this capability  -just check to make sure a GBA isn't running
    usleep(10000);
  }

  g2o::Trajectory traj_g2o = trajectories["SLAM"]->convertToG2O();
  ImagingBundleAdjustment imgBA(maps["Imaging"], trajectories["Imaging"].get(), traj_g2o, optParams );
  imgBA.Run();

  mpLocalMapper->Release();

}

} //namespace ORB_SLAM
