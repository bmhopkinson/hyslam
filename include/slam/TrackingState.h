#ifndef TRACKINGSTATE_H_
#define TRACKINGSTATE_H_

#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>

#include <TrackingStrategy.h>

namespace HYSLAM{
    class LocalMapping;

class TrackingState{
public:
    TrackingState(){}
    virtual ~TrackingState(){};
    virtual bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory) = 0; //signature mimics TrackingStrategy
    virtual bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory) = 0;
    KeyFrame* newKeyFrame(Frame &current_frame, Map* pMap, LocalMapping* pLocalMapper, unsigned int last_keyframe_id, bool force); //template method using needNewKeyFrame, createNewKeyFrame

private:
    virtual bool needNewKeyFrame(Frame &current_frame, Map* pMap, LocalMapping* pLocalMapper, unsigned int last_keyframe_id, bool force) = 0;
    virtual KeyFrame* createNewKeyFrame(Frame &current_frame, Map* pMap, LocalMapping* pLocalMapper);
};

}//end namespace

#endif