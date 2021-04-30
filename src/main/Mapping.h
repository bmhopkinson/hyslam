
#ifndef MAPPING_H
#define MAPPING_H

/*
 * class that maintains and updates Maps (keyframes, landmarks, and associations between the two)
 * runs in a separate thread and receives new keyframes (input) from tracking, incorporates them into the map, and
 * then passes them on to LoopClosing
 * Mapping is organized around running of jobs (see slam->mapping for jobs). receipt of a new KeyFrame triggers creation
 * of new jobs, some of which are mandatory others of which are optional. Mandatory jobs must be run but optional jobs can be
 * interrupted (e.g. if a new keyframe arrives). triggering job creation upon new keyframe makes sense for jobs directly connected to
 * the new keyframe but may not be the best choice for some jobs since arrival of new keyframes often comes in bursts (consider deferring optional jobs to less busy times)
 *
 * Key functionality:
 * Run() - the function that runs in a loop in a separate thread.
 *   it checks for new keyframes on the queue and if new keyframes are present it sets up the mandatory jobs defined in
 *   SetupMandatoryJobs(). the mandatory jobs are run simulatenously (in separate threads). once all mandatory jobs have completed
 *   optional jobs are created with SetupOptionalJobs() and then run with RunOptionalJobs.
 *   after this a global bundle adjustment is run if needed (break this code out into separate function). tracking is stopped during
 *   GBA b/c it takes a long time so this can only be done on post-processed data not real-time
 *   finally, execution enters a stopping point if a pause has been requested. it waits in a safe spot until asked to restart (release).
 *
 * SetupMandatoryJobs(std::vector< std::unique_ptr<MapJob> > &mandatory_jobs)
 *   sets up jobs critical to incorporation of new keyframe
 *   ProcessNewKeyFrame
 *   LandMarkCuller
 *
 * SetupOptionalJobs(std::vector< std::unique_ptr<MapJob> >  &optional_jobs);
 *   sets up optional jobs that are needed for a high quality map but don't need to be run with every new keyframe:
 *   LandMarkTriangulator
 *   LandMarkFuser
 *   LocalBundleAdjustmentJob
 *   KeyFrameCuller
 *
 * RunOptionalJobs() - runs optional jobs either single threaded
 *   or multithreaded (but this needs more work - get occasional segfaults). the optional jobs can be interrupted if needed.
 *   but if number of consecutive interrupts is tracked and interruption can be overriden if needed since the optional jobs must run
 *   periodically to obtain a high quality map.
 *
 */

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include <ORBSLAM_datastructs.h>
#include <FeatureFactory.h>
#include <InterThread.h>
#include <Tracking.h>
#include <MapJob.h>
#include <ThreadSafeQueue.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <string>


namespace HYSLAM
{

class Tracking;

class Mapping {
public:
    Mapping(std::map<std::string, Map* > &_maps, const float bMonocular,   const std::string &config_path,
            MainThreadsStatus* thread_status_, FeatureFactory* factory);

    void SetTracker(Tracking* pTracker);
    void setInputQueue(ThreadSafeQueue<KeyFrame*>* input_queue_){input_queue = input_queue_;}
    void setOutputQueue(ThreadSafeQueue<KeyFrame*>* output_queue_){output_queue = output_queue_;}

    // Main function
    void Run();

    void RequestReset();
    void Reset();
    bool isFinished();

protected:

    bool CheckNewKeyFrames();
    int  NWaitingKeyFrames();
    void ClearInputQueue();
    void SetupMandatoryJobs(std::vector< std::unique_ptr<MapJob> > &mandatory_jobs);
    void SetupOptionalJobs(std::vector< std::unique_ptr<MapJob> >  &optional_jobs);
    void RunOptionalJobs(std::vector< std::unique_ptr<MapJob> >  &optional_jobs, bool multithreaded);
    void RunGlobalBA();

    bool mbMonocular;
    cv::FileStorage config_data;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool Stop();
    bool stopRequested();
    bool interruptJobs();
    void StopTracking();
    bool clear_input_queue= false;

    void SetAcceptKeyFrames(bool flag);

    std::map<std::string, Map* > maps;

    Tracking* mpTracker;

    KeyFrame* mpCurrentKeyFrame;
    std::string curKF_cam;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    MainThreadsStatus* thread_status;
    ThreadSafeQueue<KeyFrame*>* input_queue;
    ThreadSafeQueue<KeyFrame*>* output_queue;
    int max_input_queue_length = 2;

    int interrupt_threshold = 1;
    int N_optional_jobs_stopped = 0;
    int max_optional_jobs_stopped = 3;
    FeatureFactory* feature_factory;

    long unsigned int nKFs_created = 0; // used for determining when to do global BA
    bool bNeedGBA = false; //indicates global BA needed
    long unsigned int  lastGBAKF = 0;

    std::ofstream flocalmap;

};

} //namespace ORB_SLAM

#endif // MAPPING_H
