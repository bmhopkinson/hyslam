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
#include "GlobalBundleAdjustment.h"
#include <MapPointDB.h>
#include "g2o/types/sba/Trajectory_g2o.h"
#include <mutex>

namespace HYSLAM
{

    Mapping::Mapping(std::map<std::string, Map* > &_maps, const float bMonocular, const std::string &config_path,
                     MainThreadsStatus* thread_status_, FeatureFactory* factory):
    mbMonocular(bMonocular), mbResetRequested(false), maps(_maps), thread_status(thread_status_), feature_factory(factory)
{
        std::cout << "mapping config path " << config_path << std::endl;
    config_data = cv::FileStorage(config_path ,cv::FileStorage::READ );
    interrupt_threshold = config_data["Interrupt_if_NWaiting"];

    std::string flocalmap_name = "./run_data/localmapping_data.txt";
    flocalmap.open(flocalmap_name);

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

        //WOULD LIKE TO ENABLE true flushing of queue THIS BUT REQUIRES MORE WORK - CURRENTLY ONCE KEYFRAMES are passed ot mapping they must at least initially be incorporated into the map b/c KF,Frame,mpt assocaitons have been made - can later be culled, but must initially be incorporated
        if(clear_input_queue){  //clears all EXCEPT most recent keyframe - trigerred if optional jobs keep getting interrupted or input queue is too long
            std::cout << "clearing mapping queue" << std::endl;
            StopTracking();
        }

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
            //   std::cout << "mapping: nKFs_created: " << nKFs_created << "\t lastGBAKF: " << lastGBAKF << "\t GBAinterval: " << optpar.GBAinterval << "\t time_for_GBA: " << time_for_GBA <<std::endl;
               if(!optpar.realtime && time_for_GBA ){
                  bNeedGBA = true;
                  std::cout << "need GBA = true" << std::endl;
               }

              if(bNeedGBA){
                  std::cout << "about to stop tracking for GBA" << std::endl;
                  StopTracking();

                  if(!CheckNewKeyFrames() && !stopRequested()){
                      std::cout << "running Global Bundle Adjustement !!!" << std::endl;
                       RunGlobalBA();
                       bNeedGBA = false;
                       lastGBAKF = nKFs_created;
                  } else {
                      std::cout << "aborted Global Bundle Adjustment " << std::endl;
                  }
                thread_status->tracking.setRelease(true);
               }

            }
            flocalmap << std::endl;
            output_queue->push(mpCurrentKeyFrame);
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
                while(input_queue->size() > 0){
                    input_queue->pop();
                }
                thread_status->mapping.setQueueLength(input_queue->size() );


                std::cout << "Local Mapping RELEASE" << std::endl;
            }

            if(CheckFinish())
            {
                std::cout << "Mapping - Finish requested" << std::endl;
                break;
            }
        }

        if(clear_input_queue && !CheckNewKeyFrames()) {  //release tracking
            std::cout << "starting tracking after clearing mapping queue" << std::endl;
            thread_status->tracking.setRelease(true);
            clear_input_queue = false;
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
        mpCurrentKeyFrame = input_queue->pop();
        thread_status->mapping.setQueueLength(input_queue->size());
        curKF_cam = mpCurrentKeyFrame->camera.camName;
        nKFs_created++;
     //   std::cout << "Mapping working on KF: " << mpCurrentKeyFrame->mnId<< std::endl;
      //  std::cout << "Mapping new mapping_queue size:  "<< input_queue->size() << std::endl;

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
            std::make_unique<LandMarkTriangulator>(mpCurrentKeyFrame, maps[curKF_cam],
                                                   &mlpRecentAddedMapPoints, lmtriang_params, feature_factory, flocalmap )
    );

    LandMarkFuserParameters lmfuser_params(config_data["Jobs"]["LandMarkFuser"]);
    optional_jobs.push_back(
            std::make_unique<LandMarkFuser>(mpCurrentKeyFrame, maps[curKF_cam], lmfuser_params, feature_factory,  flocalmap )
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

bool completed_jobs = true; //will get set false if aborted
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
        if ( interruptJobs() ) {
            abort = true;
            completed_jobs = false;
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
        if ( interruptJobs()) {
            std::cout << "stopping optional jobs" << std::endl;
            completed_jobs = false;
           break;
        } else {   //run job
            std::thread job_thread(&MapJob::run, (*job).get());
            job_thread.join();
        }
    }
    thread_status->mapping.setInterrupt(false);
}
if(completed_jobs){
    N_optional_jobs_stopped = 0;
} else {
    N_optional_jobs_stopped++;
}

}


bool Mapping::CheckNewKeyFrames() {
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    if(input_queue->size() > max_input_queue_length){
        clear_input_queue = true;
    }
    return (input_queue->size() > 0);
}

int  Mapping::NWaitingKeyFrames(){
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return input_queue->size();
}

void Mapping::ClearInputQueue(){
    KeyFrame* pKF_recent = input_queue->back();
    input_queue->clear();
    input_queue->push(pKF_recent);
    thread_status->mapping.setQueueLength(input_queue->size());
    std::cout << "CLEARING mapping input queue" << std::endl;
}

bool Mapping::Stop()
{
    bool res = false;
    if(stopRequested() ) {
        if(thread_status->mapping.isStoppable()) {
            thread_status->mapping.setIsStopped(true);
            std::cout << "Local Mapping STOP" << std::endl;
            res = true;
        } else {
            std::cout << "MApping stop requested, but mapping isn't stoppable" << std::endl;
        }
    }
    return res;
}

bool Mapping::stopRequested()
{
   return thread_status->mapping.isStopRequested();
}

void Mapping::SetAcceptKeyFrames(bool flag)
{
    thread_status->mapping.setAcceptingInput(flag);
}


void Mapping::StopTracking(){
    thread_status->tracking.setStopRequested(true);
    // Wait until Tracking has effectively stopped
    while(!thread_status->tracking.isStopped())
    {
        thread_status->tracking.setStopRequested(true); //someone else may have cleared the initial request - so keep putting it in
        usleep(1000);
    }
}

bool Mapping::interruptJobs()
{

    bool interrupt = (NWaitingKeyFrames() > interrupt_threshold || stopRequested() || thread_status->mapping.isInterrupt());
    if(N_optional_jobs_stopped > max_optional_jobs_stopped){ //suppress interrupt if optional jobs have not been able to complete in succession. likely due to backed up queue - only option is to empty it
        interrupt = false;
        clear_input_queue = true;
    }
    return interrupt;
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
       // mlNewKeyFrames.clear();
        while(input_queue->size() > 0 ){
            input_queue->pop();
        }
        thread_status->mapping.setQueueLength(input_queue->size());
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
