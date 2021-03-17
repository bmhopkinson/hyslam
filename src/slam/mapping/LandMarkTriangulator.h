#ifndef LANDMARKTRIANGULATOR_H_
#define LANDMARKTRIANGULATOR_H_

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Triangulator.h>
#include <FeatureFactory.h>

#include <iostream>

#include <list>

namespace HYSLAM {
    class LandMarkTriangulator : public MapJob {
    public:
        LandMarkTriangulator(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, LandMarkTriangulatorParameters params_,
                             FeatureFactory* factory, std::ofstream &log_ );
        void run();
    private:
        LandMarkTriangulatorParameters params;
        Triangulator triangulator;
        KeyFrame* pKF;
        Map* pMap;
        FeatureFactory* feature_factory;
        std::list<MapPoint*>* new_mpts;  //not ideal
        std::ofstream* log;

    };
}//end namespace

#endif