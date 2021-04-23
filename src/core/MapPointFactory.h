
#ifndef MAPPOINTFACTORY_H_
#define MAPPOINTFACTORY_H_

/*
 * factory for creating mappoints.
 * this probably isnt' necessary right now b/c "temporal" mappoints associated with frames (2nd constructor) aren't used
 * MapPoint* construct(const cv::Mat &Pos, KeyFrame *pKF, int idx) - standard construtor for persistent mappoint associated to KeyFrame, doesn't do much - most work is done by MapPointDB
 * MapPoint* construct(const cv::Mat &Pos, Frame *F, int idx) - alternative constructor for temporary mappoint linked only to a Frame - does more work adding Descriptor, etc b/c this MapPoint is not in the MapPointDB
 */

#include <MapPoint.h>
#include <Frame.h>
#include <KeyFrame.h>

#include <opencv2/core/core.hpp>

namespace HYSLAM{
    
class MapPointFactory{
    public:
      MapPointFactory();
      MapPoint* construct(const cv::Mat &Pos, KeyFrame *pKF, int idx);
      MapPoint* construct(const cv::Mat &Pos, Frame *F, int idx);
    private:
};

}
#endif
