#ifndef TRACKLOCALMAP_H_
#define TRACKLOCALMAP_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <MapPoint.h>
#include <set>

namespace ORB_SLAM2{

    class TrackLocalMap : public TrackingStrategy{
    public:
        TrackLocalMap(optInfo optimizer_info_, const TrackLocalMapParameters &params_ );
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap_, Trajectory* trajectory);
    private:
        Frame* pcurrent_frame;
        Map* pMap;
        std::set<MapPoint*> local_map_points;
        std::set<KeyFrame*> local_key_frames;
        optInfo optimizer_info;
        TrackLocalMapParameters params;

        void UpdateLocalMap();
        void SearchLocalPoints();
        void UpdateLocalKeyFrames();
        void UpdateLocalPoints();

    };

}//end namespace
#endif