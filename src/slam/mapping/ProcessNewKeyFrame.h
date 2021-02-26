#ifndef PROCESSNEWKEYFRAME_H_
#define PROCESSNEWKEYFRAME_H_

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>

namespace HYSLAM{
 class ProcessNewKeyFrame : public MapJob{
 public:
     ProcessNewKeyFrame(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, ProcessNewKeyFrameParameters params_);
     void run();
 private:
     ProcessNewKeyFrameParameters params;
     KeyFrame* pKF;
     Map* pMap;
     std::list<MapPoint*>* new_mpts;  //not ideal

     //functions

 };

}
#endif