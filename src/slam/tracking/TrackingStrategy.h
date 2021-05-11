#ifndef TRACKINGSTRATEGY_H_
#define TRACKINGSTRATEGY_H_

/*
 * abstract base class for tracking strategies - ways to estimate pose of current_frame by finding associations
 *  between landmarks in map and image features
 *  only function:
 *  int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory)
 *     implemented in derived classes - takes current_frame and attempts to estimate it's pose and associations with landmark
 *     using previous frames (FrameBuffer data), a reference keyframe pKF, the relevant map (pMap) and trajectory.
 *     returns the number of landmarks tracked.
 */

#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>

#include <vector>

namespace HYSLAM{

using FrameBuffer = std::vector<Frame>;
//using KeyFrameBuffer = std::vector<KeyFrame*>;

class TrackingStrategy{
public:
    TrackingStrategy(){};
    virtual ~TrackingStrategy(){};
    virtual int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory) = 0; //ugly but i think as good as it can get
private:


};

}

#endif
