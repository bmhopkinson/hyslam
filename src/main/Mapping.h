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

#ifndef MAPPING_H
#define MAPPING_H

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
 //   int max_input_queue_length = 2;

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
