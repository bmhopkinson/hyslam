#ifndef TRACKMOTIONMODEL_H_
#define TRACKMOTIONMODEL_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <FeatureFactory.h>

namespace HYSLAM{

    class TrackMotionModel : public TrackingStrategy{
    public:
        TrackMotionModel(optInfo optimizer_info_, const TrackMotionModelParameters &params_, FeatureFactory* factory);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
    private:
        optInfo optimizer_info;
        TrackMotionModelParameters params;
        FeatureFactory* feature_factory;
    };

}//end namespace
#endif