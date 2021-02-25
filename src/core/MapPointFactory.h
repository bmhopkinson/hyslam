
#ifndef MAPPOINTFACTORY_H_
#define MAPPOINTFACTORY_H_

#include <MapPoint.h>
#include <Frame.h>
#include <KeyFrame.h>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2{
    
class MapPointFactory{
    public:
      MapPointFactory();
      MapPoint* construct(const cv::Mat &Pos, KeyFrame *pKF, int idx);
      MapPoint* construct(const cv::Mat &Pos, Frame *F, int idx);
    private:
};

}
#endif
