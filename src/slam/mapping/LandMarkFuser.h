#ifndef LANDMARKFUSER_H_
#define LANDMARKFUSER_H_

/*
 * a map job that fuses redundant landmarks and attempts to add additional landmark observations to keyframes.
 *
 * run() - main function, can be run in a separate thread (in principle, but need to do more checking
 * b/c at least one of the optional map jobs segfaults periodically when run concurrently)
 * considers keyframes in covisibility graph of pKF_ and some secondary neighbors in the covisibility graph of those keyframes  -
 * all "local keyframes". attempts to fuse landmarks viewed in pKF_ (the focal keyframe) with each local keyframe ().
 * conversely, takes landmarks viewed in each local keyframe and attempts to fuse them into pKF_ (the focal keyframe).
 *
 *
 * fuse_mappoints(KeyFrame* pKFfuse,  std::map<std::size_t, MapPoint*> &fuse_matches)
 *  takes fuse_matches determined by FeatureMatcher::Fuse() and merges them with pKFfuse.
 *  the fuse_matches consist of a map of feature indexes in pKFfuse and newly suggested landmark matches.
 *  this function runs through each of these matches and if there is no current landmark match to idx, creates an association.
 *  if there is a current landmark match to idx, it determines whether the existing landmark or the fuse candidate has more
 *  current observations. whichever has less observations is replaced (replaceMapPoint() ) by the landmark with more observations.
 */

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <FeatureFactory.h>
#include <Map.h>
#include <iostream>

namespace HYSLAM{
class LandMarkFuser : public MapJob {
public:
    LandMarkFuser(KeyFrame *pKF_, Map *pMap_, LandMarkFuserParameters params_, FeatureFactory* factory,  std::ofstream &log_);
    void run();

private:
    LandMarkFuserParameters params;
    KeyFrame* pKF;
    Map* pMap;
    FeatureFactory* feature_factory;
    std::ofstream* log;

    int fuse_mappoints(KeyFrame* pKFfuse,  std::map<std::size_t, MapPoint*> &fuse_matches);
};

}//end namespace
#endif