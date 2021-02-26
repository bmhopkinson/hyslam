#ifndef TRACKINGSTRATEGY_H_
#define TRACKINGSTRATEGY_H_

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
    virtual int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory) = 0; //ugly but i think as good as it can get
private:


};

}

#endif
