#ifndef TRACKINGSTATE_H_
#define TRACKINGSTATE_H_

#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>
#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>

#include <iostream>

namespace HYSLAM{
  //  class Mapping;

class TrackingState{
public:
    TrackingState(std::ofstream &log, MainThreadsStatus* thread_status_);
    virtual ~TrackingState(){};
    virtual bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,   std::map< std::string, std::unique_ptr<Trajectory> > &trajectories) = 0; //signature mimics TrackingStrategy
    virtual bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories) = 0;
    std::vector<KeyFrame*> newKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force); //template method using needNewKeyFrame, createNewKeyFrame
protected:
    std::ofstream* pftracking;
    MainThreadsStatus* thread_status;
private:
    virtual bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force) = 0;
    virtual std::vector<KeyFrame*> createNewKeyFrame(Frame &current_frame, Map* pMap);

};

}//end namespace

#endif