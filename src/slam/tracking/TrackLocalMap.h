#ifndef TRACKLOCALMAP_H_
#define TRACKLOCALMAP_H_

/*
 * TrackingStrategy primarily used to refine pose estimate by first running SearchByProjection on local points to
 *   add further landmark matches and then running PoseOptimization on current_frame
 *
 * track() - first finds landmarks that might be viewed in the current_frame (local_map_points) in UpdateLocalMap(). then attempts to
 *    find additional landmark matches between these local_map_points and current frame using FeatureMatcher::SearchByProjection in  SearchLocalPoints(),
 *    Next runs PoseOptimization to refine pose estimate based on landmark to image feature correspondences.
 *    subsequently removes outlier landmark associations and returns number of remaining associations.
 *
 *  UpdateLocalMap() - calls UpdateLocalKeyFrames(), UpdateLocalPoints().
 *
 *  UpdateLocalKeyFrames() - first finds all keyframes that observe landmarks tracked in current_frame and adds them to local_key_frames.
 *       next, finds additional keyframes connected in covisibility graph (number is settable with parameters) and keyframes connected
 *       by spanning tree (this is likely overkill) and adds them to local_key_frames
 *
 *  UpdateLocalPoints() - adds all landmarks viewed in local_key_frames to local_map_points
 *
 *  SearchLocalPoints() - runs SearchByProjection on current frame and local_map_points to attempt to find additional
 *     landmark to image feature matches. excludes landmarks already associated with the current_frame
 *
 */

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <MapPoint.h>
#include <FeatureFactory.h>
#include <set>

namespace HYSLAM{

    class TrackLocalMap : public TrackingStrategy{
    public:
        TrackLocalMap(optInfo optimizer_info_, const TrackLocalMapParameters &params_, FeatureFactory* factory );
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap_, Trajectory* trajectory);
    private:
        Frame* pcurrent_frame;
        Map* pMap;
        std::set<MapPoint*> local_map_points;
        std::set<KeyFrame*> local_key_frames;
        optInfo optimizer_info;
        TrackLocalMapParameters params;
        FeatureFactory* feature_factory;

        void UpdateLocalMap();
        void SearchLocalPoints();
        void UpdateLocalKeyFrames();
        void UpdateLocalPoints();

    };

}//end namespace
#endif