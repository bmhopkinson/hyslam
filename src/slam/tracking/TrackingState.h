#ifndef TRACKINGSTATE_H_
#define TRACKINGSTATE_H_

#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>
#include <TrackingStrategy.h>
#include <InterThread.h>

#include <iostream>

/*
 * abstract base class defining interface for TrackingState classes
 * the derived classes implement different tracking behaviors dependent on quality of tracking
 * the interfaces are somewhat awkward and tailored to meet needs of current tracking state classes - may need to generalize.
 *
 * key functions:
 * initialPoseEstimation() - attempts initial pose estimate for current_frame based on data in the FrameBuffer (previous frames)
 *    pKF (a reference keyframe), the map, and trajectory data (for this camera or others)
 *
 * refinePoseEstimate() - attempts to refine pose estimate if initial pose estimate was successful. uses the same information available
 *    for the initial pose estimation
 *
 * newKeyFrame() - determines if new keyframes (possibly multiple, e.g. monocular initialization) are needed  by calling needNewKeyFrame(). if they are the new keyframes
 *    are created and returned by the function. this function is implemented in the base class and currently not declared virtual
 *
 * needNewKeyFrame() - function that determines if a new keyframe should be inserted. state specific.
 *
 * createNewKeyFrame() - function that actually constructs new keyframe. implemented here in base class, but can be overriden.
 *    for monocular cameras, the function
 *   simply constructs a keyframe from the current_frame. for stereo cameras also creates new landmarks using the map - this is a little weird b/c the 
 *   the keyframe isn't officially in the map yet. probably preferable to do this in mapping. 
 * 
 */

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