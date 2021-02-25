#ifndef LANDMARKCULLER_H_
#define LANDMARKCULLER_H_

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>

#include <iostream>

#include <list>

namespace ORB_SLAM2 {
    class LandMarkCuller : public MapJob {
    public:
        LandMarkCuller(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, LandMarkCullerParameters params_, std::ofstream &log_);
        void run();
    private:
        LandMarkCullerParameters params;
        KeyFrame* pKF;
        Map* pMap;
        std::list<MapPoint*>* new_mpts;  //not ideal
        std::ofstream* log;


    };
}//end namespace

#endif