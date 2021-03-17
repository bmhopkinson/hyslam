#ifndef LANDMARKFUSER_H_
#define LANDMARKFUSER_H_

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