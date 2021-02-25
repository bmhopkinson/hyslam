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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "MonoEstimator.h"
#include <MonoInitializer.h>
#include "MapDrawer.h"
#include "System.h"
#include "ORBSLAM_datastructs.h"
#include "Tracking_datastructs.h"
#include "Trajectory.h"
#include <Camera.h>
#include <SensorData.h>
#include <MapPointFactory.h>

#include <TrackingState.h>

#include <mutex>
#include <map>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;


class Tracking
{

public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    Tracking(System* pSys, ORBVocabulary* pVoc, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer* pMapDrawer, std::map<std::string, Map* > &_maps,
             const string &strSettingPath);
    ~Tracking();

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    eTrackingState GetCurrentTrackingState() {return mState["SLAM"];};
    void RunImagingBundleAdjustment();


    // other getters setters
    KeyFrame* GetReferenceKF() { return mpReferenceKF["SLAM"] ;}

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void LoadCalibration(const cv::FileNode &camera, std::string cam_name);
    void InitializeDataStructures(std::string cam_name);
    cv::Mat PreProcessImg(cv::Mat &img, bool mbRGB, float fscale);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    // Thread Synch - for stopping when post-processing results - not for realtime use
    void RequestStop();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();


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
    std::map< std::string, Trajectory> trajectories;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    //optimizer parameters
    optInfo optParams;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();
    void SetupStates();

    void UpdateLastFrame();
    KeyFrame* determineReferenceKeyFrame(Frame* pcurrent_frame);
    bool Relocalization();
    void UpdateTrajectory(bool tracking_success);

    //Handlers
    int HandlePostInit(KeyFrame* pKFcurrent, Map* pMap,std::string cam_name );
    void HandlePostTrackingSuccess();

    //Dataloaders
    void LoadSettings(std::string settings_path, ORBextractorSettings &ORBext_settings);


    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;
    bool bOK;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    // state data
    std::map<std::string, TrackingState*> state;  //camera to state map
    std::map<std::string, TrackingState*> state_options; //state name to state pointer map

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
 //   KeyFrameDatabase* mpKeyFrameDB;

    // Initalization
    int stereoInitFeatures;

    //Local Map
    std::map<std::string, KeyFrame*> mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    

    // System
    System* mpSystem;

    //Drawers
    Viewer* mpViewer;
    std::map<std::string, FrameDrawer*> mpFrameDrawers;
    MapDrawer* mpMapDrawer;

    //Map
    std::map<std::string, Map* > maps;

    //camera data - combine all this into a camera info structure
    std::map<std::string, Camera> cam_data;

    //stopping - for postprocessing only
    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;


    //Last Frame, KeyFrame and Relocalisation Info
    std::map<std::string, Frame> mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    std::ofstream fout;
    std::ofstream ftracking; 
};

} //namespace ORB_SLAM

#endif // TRACKING_H
