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
#include <Tracking.h>
#include <MapJob.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>


namespace HYSLAM
{

class Tracking;
class LoopClosing;
class Map;

class Mapping
{
public:
    Mapping(std::map<std::string, Map* > &_maps, const float bMonocular,   const string &config_path);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
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
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    std::map<std::string, Map* > maps;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;
    std::string curKF_cam;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;
    bool abortJobs;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    int miLBAcalls = 0; //local BA calls- do global BA periodically
    int nKFs_created = 0; // used for determining when to do global BA
    bool bNeedGBA = false; //indicates global BA needed

    std::ofstream flocalmap;

};

} //namespace ORB_SLAM

#endif // MAPPING_H
