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



#ifndef TRACKING_H
#define TRACKING_H

/*
 * class that performs frame to frame tracking with respect to a Map
 * organized as a state machine where the attempted tracking method varies depending on state (normal, initalization, relocalization)
 * and potentially camera. tracking states and tracking strategies are defined in slam->tracking pacakge.
 * runs in a separate thread and receives extracted feature data per frame from ImageProcessing. attempts to determine pose of frame
 * and landmark to feature correspondences based on various tracking strategies. after successful tracking determines if a new
 * keyframe is needed - if so it creates one from the existing frame and puts it onto the output queue for mapping to handle.
 * also maintains the frame by frame trajectory for each camera.
 *
 * key data:
 *
 * currentFrame - represents the current image holding its extracted features, correspondences to landmarks, and pose
 * maps, camera_data - self explanatory, one per camera
 * state - current tracking state, one per camera
 * state_options - tracking states available for use, one set per camera
 * trajectories - trajectories representing the per frame path of each camera
 *
 * key functions:
 * constructor - sets pointers to various resources (some of which should be refactored to eliminate - drawers, system)
 * loads settings from strSettingPath using internal function LoadSettings(std::string settings_path) which loads optimizer settings,
 * , finds tracking state and strategies settings file, and sets up data structures. Finally, calls SetupStates() that uses tracking configuration data
 * to set up tracking states and strategies for each camera - currently must have the following states: Normal, Initialize, Relocalize (but would like
 * to make this more generic)
 *
 * Run() - main looping function run in a thread. looks for image feature data on the input queue and if available runs track().
 * then tests if a stop has been requested and waits in a safe area if one has been requested until asked to resume (release)
 *
 * track() - creates Frame from image feature data and calls internal function _Track()_
 * _Track()_ - attempts to track the Frame (i.e. determine its pose and landmark to feature correspondences) with calls
 *  to intialPoseEstimatation() and if successful refinePoseEstimate() implemented by the current tracking state. if tracking was successful
 *  decides whether a new keyframe needs to be inserted by calling newKeyFrame(), again implemented by the current tracking state (and so state dependent).
 *  if a new keyframe is created it is passed on to mapping via the output queue.
 *  adds current frame to the cameras trajectory. updates viewer (FrameDrawer; this should be implemented differently to reduce dependency)
 *  and finally determines next state and sets it in "state" - currently this logic is hard coded but would like to make it more flexible (run time loadable or implemented by states)
 *
 *
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include <ORBSLAM_datastructs.h>
#include <InterThread.h>
#include "Frame.h"
//#include "ORBVocab.h"
#include "FeatureExtractor.h"
#include <FeatureFactory.h>
#include <MonoInitializer.h>
#include "MapDrawer.h"
#include "System.h"
#include "Tracking_datastructs.h"
#include "Trajectory.h"
#include <Camera.h>
#include <SensorData.h>
#include <ThreadSafeQueue.h>

#include <TrackingState.h>

#include <mutex>
#include <string>
#include <memory>
#include <map>
#include <iostream>


namespace HYSLAM
{

class Tracking
{
//
public:

    Tracking(System* pSys, FeatureVocabulary* pVoc, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer* pMapDrawer, std::map<std::string, std::shared_ptr<Map> > &_maps,
             std::map<std::string, Camera > cam_data_, const std::string &strSettingPath, MainThreadsStatus* thread_status_, FeatureFactory* factory);
    ~Tracking();

    void Run();
    cv::Mat track(ImageFeatureData &track_data);

 //   void SetLocalMapper(Mapping* pLocalMapper);
    void SetViewer(Viewer* pViewer);
    void setInputQueue(ThreadSafeQueue<ImageFeatureData>* input_queue_){input_queue = input_queue_;}
    void setOutputQueue(ThreadSafeQueue<KeyFrame*>* output_queue_){output_queue = output_queue_;}
    eTrackingState GetCurrentTrackingState() {return mState["SLAM"];};
    eTrackingState GetCurrentTrackingState(std::string cam_name) {return mState[cam_name];};

    // other getters setters
    KeyFrame* GetReferenceKF() { return mpReferenceKF["SLAM"] ;}

    // Load new settings
    void InitializeDataStructures(std::string cam_name);

public:

    std::map<std::string, eTrackingState> mState;

    eTrackingState mLastProcessedState;  //only used for frame drawer currently

    // Input sensor
    std::map<std::string, int> mSensor;
    std::vector<std::string> cam_types; //all cameras in the system.
    std::string cam_cur;  //current camera

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // ADDITION: Tracking state monitoring
    int nPoints = 0;
    int nObserved = 0;
    double PercentObserved() { return (nPoints > 0) ? static_cast<double>(nObserved)/static_cast<double>(nPoints) : 0; }

    // Initialization Variables (Monocular)
    std::map< std::string, InitializerData> init_data;
    std::map< std::string, int> recent_init;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::map< std::string, std::unique_ptr<Trajectory> > trajectories;

    //optimizer parameters
    optInfo optParams;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void _Track_();
    void SetupStates();
    bool inputAvailable();

    void UpdateLastFrame();
    KeyFrame* determineReferenceKeyFrame(Frame* pcurrent_frame);

    //Handlers
    int HandlePostInit(KeyFrame* pKFcurrent, Map* pMap,std::string cam_name );
    void HandlePostTrackingSuccess();

    //Dataloaders
    void LoadSettings(std::string settings_path);
    cv::FileStorage config_data;

    //Other Thread Pointers
 //   Mapping* mpLocalMapper;

    // state data
    std::map<std::string, TrackingState*> state;  //camera to state map
    std::map<std::string, std::map<std::string, TrackingState*> > state_options; //state name to state pointer map

    //BoW
    FeatureVocabulary* mpORBVocabulary;
    FeatureFactory* feature_factory;

    // Initalization
    int stereoInitFeatures;
    bool slam_ever_initialized = false;

    //Local Map
    std::map<std::string, KeyFrame*> mpReferenceKF;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    //Drawers
    Viewer* mpViewer;
    std::map<std::string, FrameDrawer*> mpFrameDrawers;
    MapDrawer* mpMapDrawer;

    //Map
    std::map<std::string, std::shared_ptr<Map> > maps;

    //camera data
    std::map<std::string, Camera> cam_data;

    //stopping - for postprocessing only
    MainThreadsStatus* thread_status;
    bool Stop();
    ThreadSafeQueue<ImageFeatureData>* input_queue;
    ThreadSafeQueue<KeyFrame*>* output_queue;

    //Last Frame, KeyFrame and Relocalisation Info
    std::map<std::string, Frame> mLastFrame;
    unsigned int mnLastKeyFrameId;

    std::ofstream fout;
    std::ofstream ftracking; 
};

} //namespace ORB_SLAM

#endif // TRACKING_H
