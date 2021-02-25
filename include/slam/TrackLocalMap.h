#ifndef TRACKLOCALMAP_H_
#define TRACKLOCALMAP_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <MapPoint.h>
#include <set>

namespace ORB_SLAM2{

    class TrackLocalMap : public TrackingStrategy{
    public:
        TrackLocalMap(optInfo optimizer_info_);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap_, const Trajectory &trajectory);
    private:
        optInfo optimizer_info;
        Frame* pcurrent_frame;
        Map* pMap;
        std::set<MapPoint*> local_map_points;
        std::set<KeyFrame*> local_key_frames;

        void UpdateLocalMap();
        void SearchLocalPoints();
        void UpdateLocalKeyFrames();
        void UpdateLocalPoints();

    };

}//end namespace
#endif