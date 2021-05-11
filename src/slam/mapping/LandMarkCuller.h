#ifndef LANDMARKCULLER_H_
#define LANDMARKCULLER_H_

/*
 * a map job that removes redundant landmarks from a map
 *
 * run() - main function, can be run in a separate thread
 * checks new_mpts_ to see if the landmarks should be removed. several criteria are considered:
 * 1. is the landmark protected? if so don't remove (cull)
 * 2. if a certain number (specified in LandMarkCullerParameters) of keyframes have passed since the keyframe in which the landmark
 *  was created, checks to see how many times the landmark has been observed. if the number of observations are below a threshold (set in parameters)
 *  then remove the landmark from the map.
 *  once the landmark has survived for the keyframe grace period and been observed in a sufficient number of keyframes, it is removed from new_mpts and persists.
 *
 *
 */

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>

#include <iostream>

#include <list>

namespace HYSLAM {
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