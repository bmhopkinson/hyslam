
#ifndef SYSTEM_H
#define SYSTEM_H

/*
 * class representing the SLAM system to be used by external applications/libraries.
 * also sets up the main objects and threads and handles shutdown (and data export - but that should be moved to another class)
 * Key functionality:
 * System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true)
 *  - initializes the system using the settings in strSettingsFile (strVocFile is LEGACY - not used, should be removed), sensor is the SLAM camera type
 *    sets up FeatureFactory, loads camera data and other settings, sets up Tracking, Mapping, and LoopClosing threads as well as
 *    interthread queues. sets up Viewer is bUseViewer=true
 *
 *
 * TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const Imgdata &img_info, const SensorData &sensor_data)
 *   note: doesn't return camera pose right now
 *   initiates processing of the stereoframe.
 *   hands off stereoimage data to ImageProcessing (which then passes to Tracking, etc), waits to return if tracking queue is backing up
 *
 * TrackMonocular(const cv::Mat &im, const Imgdata &img_info, const SensorData &sensor_data)
 *   note: doesn't return camera pose right now
 *   initiates processing of the monocular frame.
 *   hands off image data to ImageProcessing (which then passes to Tracking, etc), waits to return if tracking queue is backing up
 *
 * RunImagingBundleAdjustment() - runs imaging bundle adjustment to finalize alignment of imaging camera data after end of SLAM run
 *
 * void Shutdown() - shutsdown all threads (Tracking, Mapping, LoopClosing) in preparation to exit 
 *
 * ExportCOLMAP(const std::string &foldername)
 *   exports SLAM data in format for sparse reconstrution import for COLAMP
 *
 * SaveKeyFramesAgisoft(const std::string &filename) - exports keyframes and associated camera intrinsics in Agisoft
 * metashape format for use with metashape
 */

#include <Tracking.h>
#include <Mapping.h>
#include <ImageProcessing.h>
#include "LoopClosing.h"
#include <InterThread.h>
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Viewer.h"
#include "Map.h"
//#include "ORBVocab.h"
#include "ORBSLAM_datastructs.h"
#include <Tracking_datastructs.h>
#include <SensorData.h>
#include <ThreadSafeQueue.h>
#include <FeatureVocabulary.h>
#include <FeatureFactory.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <thread>
#include <unistd.h>
#include <map>
#include <memory>

namespace HYSLAM
{

class Viewer;
class FrameDrawer;
class Tracking;
class Mapping;
class LoopClosing;
class ImageProcessing;

using KeyFrameExportData = std::vector< std::pair<int, std::string > >;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const Imgdata &img_info, const SensorData &sensor_data);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const Imgdata &img_info, const SensorData &sensor_data);

    void RunImagingBundleAdjustment(); //after completion of SLAM run call to align imaging cameras

    std::map<std::string, Camera>  getCameras(){return cam_data;};


    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
//    bool MapChanged();

    // Reset the system (clear map)
    void Reset();
    void RequestReset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();


    //for BH mapping
    void SaveTrajectoryMapping(const std::string &filename);

    void ExportCOLMAP(const std::string &foldername);
    void SaveKeyFramesAgisoft(const std::string &filename);
    std::map< std::string, KeyFrameExportData > KeyFramesInfoForExport();
    std::vector<int> ValidImagingKeyFrames();


    // output for debugging whatever is currently an issue
//    void SaveTrajectoryDebugging(const std::string &filename);

    // TODO: Save/Load functions
    // SaveMap(const std::string &filename);
    // LoadMap(const std::string &filename);
    void SaveMap(const std::string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();


    // ADDITIONS: Tracking state monitoring

    // Check if tracking is working
    bool TrackingOk();

    // Check if tracking is lost
    bool TrackingLost();

    // Check if tracking is ready to run
    bool TrackingReady();

    // Check if tracking is waiting for images
    bool TrackingNeedImages();

    // Check if tracking is initialized
    bool TrackingInitialized();

    // Get percentage of previously observed points compared to total in view
    // Can be useful as an indicator of tracking quality after relocalization
    double PercentObserved();

private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    std::unique_ptr<FeatureFactory> feature_factory;
    FeatureVocabulary* mpVocabulary;
   // void LoadVocabulary(const std::string vocab_file, FeatureVocabulary* vocab);

    void LoadSettings(std::string settings_path);

    // camera-specific Map structures that stores the pointers to all KeyFrames and MapPoints.
    std::map<std::string, Map*> maps;

    //data shared between threads
    std::unique_ptr< MainThreadsStatus > thread_status;
    std::unique_ptr< ThreadSafeQueue<ImageFeatureData> > tracking_queue;
    std::unique_ptr< ThreadSafeQueue<KeyFrame*> > mapping_queue;
    std::unique_ptr< ThreadSafeQueue<KeyFrame*> > loopclosing_queue;

  //  std::unique_ptr< ImageFeatureData > tracking_queue;
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    Mapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    //Image Processer
    ImageProcessing* mpImageProcessor;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    std::map<std::string, FrameDrawer*> mpFrameDrawers;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The ImageProcessing thread "lives" in the main execution thread that creates the System object.
    std::thread* mptTracking;
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    std::map<std::string, Camera> cam_data;
    optInfo optParams;

    // Tracking state
    std::map<std::string, eTrackingState> current_tracking_state;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

std::string createImageFileName(std::string cam_name, std::string img_id, std::string file_ext);

}// namespace ORB_SLAM

#endif // SYSTEM_H
