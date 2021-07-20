#ifndef PROCESSNEWKEYFRAME_H_
#define PROCESSNEWKEYFRAME_H_

/*
 * fundamental map job that sets up the new keyframe  - MUST BE RUN FIRST
 *
 * run() incorporates the new keyframe (pKF_) into the map and does some other work:
 * 1. computes BoW (this is computational expensive so it's not done in tracking)
 * 2. takes keypoint to landmark associations stored in keyframe and registers them in the map (pMap_) - previously these associations
 *    were only known by the keyframe, as inherited from the frame it was created from.
 * 3. keyframe is added to map - this adds keyframe to KeyFrameDB (covisibility graph, spanning tree) and to list of keyframes in map
 *     seems a bit odd to do this last, may want to rethink.
 */

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>

namespace HYSLAM{
 class ProcessNewKeyFrame : public MapJob{
 public:
     ProcessNewKeyFrame(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, ProcessNewKeyFrameParameters params_);
     void run();
     std::string name(){return "ProcessNewKeyFrame";};
 private:
     ProcessNewKeyFrameParameters params;
     KeyFrame* pKF;
     Map* pMap;
     std::list<MapPoint*>* new_mpts;  //not ideal

     //functions

 };

}
#endif