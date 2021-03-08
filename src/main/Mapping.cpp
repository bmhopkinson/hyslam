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

#include <Mapping.h>
#include <MappingDataStructs.h>
#include <ProcessNewKeyFrame.h>
#include <KeyFrameCuller.h>
#include <LandMarkCuller.h>
#include <LandMarkTriangulator.h>
#include <LandMarkFuser.h>
#include <LocalBundleAdjustmentJob.h>
#include "LoopClosing.h"
#include "GlobalBundleAdjustment.h"
#include <MapPointDB.h>
#include "g2o/types/sba/Trajectory_g2o.h"
#include <mutex>

namespace HYSLAM
{

    Mapping::Mapping(std::map<std::string, Map* > &_maps, const float bMonocular, const std::string &config_path, MainThreadsStatus* thread_status_):
    mbMonocular(bMonocular), mbResetRequested(false), maps(_maps), thread_status(thread_status_)
{
        std::cout << "mapping config path " << config_path << std::endl;
    config_data = cv::FileStorage(config_path ,cv::FileStorage::READ );

    std::string flocalmap_name = "./run_data/localmapping_data.txt";
    flocalmap.open(flocalmap_name);

}

void Mapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void Mapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void Mapping::Run()
{

    while(1)
    {

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {

            std::vector< std::unique_ptr<MapJob> > mandatory_jobs;  //eventually want to turn all of this into a true WorkQueue w/ threadpools, priority queue for optional jobs...
            SetupMandatoryJobs(mandatory_jobs);
            for(auto job = mandatory_jobs.begin(); job != mandatory_jobs.end(); ++job){
                std::thread job_thread(&MapJob::run, (*job).get());
                job_thread.join();
            }

            std::vector< std::unique_ptr<MapJob> > optional_jobs;
            SetupOptionalJobs(optional_jobs);
            RunOptionalJobs(optional_jobs, false);

            if(!CheckNewKeyFrames() && !stopRequested())
            {

                 optInfo optpar = mpTracker->optParams;
               //periodically do a Global Bundle Adjustment
               bool time_for_GBA = (nKFs_created - lastGBAKF) > optpar.GBAinterval;
             //  std::cout << "mapping: nKFs_created: " << nKFs_created << "\t lastGBAKF: " << lastGBAKF << "\t time_for_GBA: " << time_for_GBA <<std::endl;
               if(!optpar.realtime && time_for_GBA ){
                  bNeedGBA = true;
                  std::cout << "need GBA = true" << std::endl;
               }

              if(bNeedGBA){
                  thread_status->tracking.setStopRequested(true);
                   // Wait until Tracking has effectively stopped
                  while(!thread_status->tracking.isStopped())
                  {
                        usleep(1000);
                  }

                  if(!CheckNewKeyFrames() && !stopRequested()){
                      std::cout << "running Global Bundle Adjustement !!!" << std::endl;
                       RunGlobalBA();
                       bNeedGBA = false;
                       lastGBAKF = mpCurrentKeyFrame->mnId ;
                  }
                thread_status->tracking.setRelease(true);
               }

            }
            flocalmap << std::endl;
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            std::cout << "Mapping Stop!!!" << std::endl;
           while(!thread_status->mapping.isRelease() && !CheckFinish())
            {
                usleep(3000);
            }

            {//relase actions
                if(isFinished())
                    return;
                thread_status->mapping.clearPostStop();
                for(std::list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
                    delete *lit;
                mlNewKeyFrames.clear();

                std::cout << "Local Mapping RELEASE" << std::endl;
            }

            if(CheckFinish())
            {
                std::cout << "Mapping - Finish requested" << std::endl;
                break;
            }
        }
        ResetIfRequested();

        if(CheckFinish()) {
            std::cout << "Mapping - Finish requested" << std::endl;
            break;
        }
        // Tracking will see that Local Mapping is NOT busy
        SetAcceptKeyFrames(true);
        usleep(3000);
    }

    SetFinish();
}

//std::vector<MapJob*> Mapping::SetupMandatoryJobs(){
void Mapping::SetupMandatoryJobs(std::vector< std::unique_ptr<MapJob> > &mandatory_jobs){
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        curKF_cam = mpCurrentKeyFrame->camera.camName;
       // std::cout << "processing new KF:" << mpCurrentKeyFrame->mnId << " from cam: " << curKF_cam << std::endl;
        mlNewKeyFrames.pop_front();
        nKFs_created++;
    }


    ProcessNewKeyFrameParameters kf_params(config_data["Jobs"]["ProcessNewKF"]);
    mandatory_jobs.push_back(
            std::make_unique<ProcessNewKeyFrame>(mpCurrentKeyFrame, maps[curKF_cam], &mlpRecentAddedMapPoints, kf_params )
            );

    LandMarkCullerParameters  lmculler_params(config_data["Jobs"]["LandMarkCuller"]);
    mandatory_jobs.push_back(
            std::make_unique<LandMarkCuller>(mpCurrentKeyFrame, maps[curKF_cam], &mlpRecentAddedMapPoints, lmculler_params, flocalmap )
            );

}

void Mapping::SetupOptionalJobs(std::vector< std::unique_ptr<MapJob> > &optional_jobs){

    LandMarkTriangulatorParameters lmtriang_params(config_data["Jobs"]["LandMarkTriangulator"]);
    optional_jobs.push_back(
            std::make_unique<LandMarkTriangulator>(mpCurrentKeyFrame, maps[curKF_cam], &mlpRecentAddedMapPoints, lmtriang_params, flocalmap )
    );

    LandMarkFuserParameters lmfuser_params(config_data["Jobs"]["LandMarkFuser"]);
    optional_jobs.push_back(
            std::make_unique<LandMarkFuser>(mpCurrentKeyFrame, maps[curKF_cam], lmfuser_params,  flocalmap )
    );

    if(maps[curKF_cam]->KeyFramesInMap()>2) {
        optInfo optpar = mpTracker->optParams;
        g2o::Trajectory traj_g2o = mpTracker->trajectories["SLAM"]->convertToG2O();

        optional_jobs.push_back(
                std::make_unique<LocalBundleAdjustmentJob>(mpCurrentKeyFrame, maps[curKF_cam], traj_g2o, optpar, flocalmap )
        );

    }

    KeyFrameCullerParameters kfculler_params(config_data["Jobs"]["KeyFrameCuller"]);
    if(curKF_cam == "SLAM") {
       // std::cout << "keyframe culler for: " << curKF_cam << std::endl;
        optional_jobs.push_back(
                std::make_unique<KeyFrameCuller>(mpCurrentKeyFrame, maps[curKF_cam], kfculler_params, flocalmap)
        );
    }

}

void Mapping::RunOptionalJobs(std::vector< std::unique_ptr<MapJob> >  &optional_jobs, bool multithreaded){
        // testing didn't show any advantage of multithreading right now - all jobs almost always finish before next keyframe arrives.
        // multithreading did seem to have somewhat more frequent segfaults but generally worked ok - so it could be possible to multithread if needed but will take some work 
if(multithreaded){
    using JobThread = std::pair<MapJob *, std::thread>;
    std::list<JobThread> active_jobs;

    //launch threads
    for (auto job = optional_jobs.begin(); job != optional_jobs.end(); ++job) {
        active_jobs.emplace_back((*job).get(),
                                 std::thread(&MapJob::run, (*job).get())
        );
    }

    bool abort = false;
    while (!active_jobs.empty()) {
        if (CheckNewKeyFrames() || stopRequested() || interruptJobs() ) {
            abort = true;
            //  std::cout << "attempting to abort jobs" << std::endl;
        }
        for (auto it = active_jobs.begin(); it != active_jobs.end(); ++it) {
            MapJob *job = it->first;
            if (abort) {
                job->abort();
            }

            if (job->finished()) {   //if job is done remove it.
                //   std::cout <<" job finished for KF: " << mpCurrentKeyFrame->mnId <<std::endl;
                std::thread &job_thread = it->second;
                job_thread.join();
                it = active_jobs.erase(it);
            }

        }
    }
    thread_status->mapping.setInterrupt(false);
}
else {
    for(auto job = optional_jobs.begin(); job != optional_jobs.end(); ++job){
        if (CheckNewKeyFrames() || stopRequested()  || interruptJobs()) {
            std::cout << "stopping optional jobs" << std::endl;
           break;
        } else {   //run job
            std::thread job_thread(&MapJob::run, (*job).get());
            job_thread.join();
        }
    }
    thread_status->mapping.setInterrupt(false);
}

}

void Mapping::InsertKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    std::cout << "mapping received new KF: " << pKF->mnId << std::endl;
   // thread_status->mapping.setInterrupt(true);
}


bool Mapping::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

bool Mapping::Stop()
{
    bool res = false;
    if(stopRequested() && !thread_status->mapping.isStoppable())
    {
        thread_status->mapping.setIsStopped(true);
        std::cout << "Local Mapping STOP" << std::endl;
        res =  true;
    }
    return res;
}

bool Mapping::isStopped()
{
  return thread_status->mapping.isStopped();
}

bool Mapping::stopRequested()
{
   return thread_status->mapping.isStopRequested();
}

void Mapping::SetAcceptKeyFrames(bool flag)
{
    thread_status->mapping.setAcceptingInput(flag);
}

bool Mapping::SetNotStop(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexStop);

    if(flag && isStopped())
        return false;
    thread_status->mapping.setStoppable(flag);

    return true;
}

bool Mapping::interruptJobs()
{
   return thread_status->mapping.isInterrupt();
}

void Mapping::RunGlobalBA(){

    std::cout << "starting global BA" << std::endl;
   mpTracker->optParams.GBAtype = 1; //periodic GBA
   KeyFrame* pRefKF = mpTracker->GetReferenceKF();
    while(pRefKF->isBad())
    {
         pRefKF = pRefKF->GetParent();
     }
    //vector<KeyFrame*> vpKFfixed = pRefKF->GetBestCovisibilityKeyFrames(4); // fix reference keyframe and 4 most covisible - this is to prevent loss of tracking upon return from GBA - ultimately may be better to deal with Tracking thread to handle this
    std::vector<KeyFrame*> vpKFfixed =  maps[curKF_cam]->getKeyFrameDB()->GetBestCovisibilityKeyFrames(pRefKF, 4);
    vpKFfixed.push_back(pRefKF);

    bool mbStopGBA = false;
    int nIter = 20;
    bool  bRobust = false;
    g2o::Trajectory traj_g2o = mpTracker->trajectories["SLAM"]->convertToG2O();
    GlobalBundleAdjustment globalBA(vpKFfixed,  0, nIter ,  bRobust, &mbStopGBA, maps[pRefKF->camera.camName], traj_g2o, mpTracker->optParams);
    globalBA.Run();

    std::cout << "finished global BA" << std::endl;

}

void Mapping::RequestReset()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            std::unique_lock<std::mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void Mapping::ResetIfRequested()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        std::cout << "Reseting Mapping due to Request" << std::endl;
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

bool Mapping::CheckFinish()
{

    return thread_status->mapping.isFinishRequested();
}

void Mapping::SetFinish()
{
    thread_status->mapping.setIsFinished(true);
    thread_status->mapping.setIsStopped(true);
    std::cout << "Mapping finished  !!!!!!" << std::endl;
}

bool Mapping::isFinished()
{
    return thread_status->mapping.isFinished();
}

} //namespace ORB_SLAM
