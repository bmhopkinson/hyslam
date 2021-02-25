#ifndef TRACKINGSTATE_H_
#define TRACKINGSTATE_H_

#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>
#include <TrackingStrategy.h>

#include <iostream>

namespace ORB_SLAM2{
    class Mapping;

class TrackingState{
public:
    TrackingState(std::ofstream &log);
    virtual ~TrackingState(){};
    virtual bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,   std::map< std::string, std::unique_ptr<Trajectory> > &trajectories) = 0; //signature mimics TrackingStrategy
    virtual bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories) = 0;
    KeyFrame* newKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force); //template method using needNewKeyFrame, createNewKeyFrame
protected:
    std::ofstream* pftracking;
private:
    virtual bool needNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force) = 0;
    virtual KeyFrame* createNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper);

};

}//end namespace

#endif