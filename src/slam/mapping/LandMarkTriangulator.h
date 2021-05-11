#ifndef LANDMARKTRIANGULATOR_H_
#define LANDMARKTRIANGULATOR_H_

/*
 *  map job that creates new landmarks by triangulation of feature matches between keyframes
 *
 *  run() - main function, can be run in a separate thread (in principle, but need to do more checking
 * b/c at least one of the optional map jobs segfaults periodically when run concurrently)
 * finds keyframes connected in covisibility graph to pKF_, considered "neighbor" keyframes.
 * attempts to triangulate new landmarks between pKF_ (focal keyframe) and each of the neighbor keyframes, as long
 * as there is a sufficient distance (baseline) between the two keyframes. First, matches features between the two keyframes
 * with a call to FeatureMatcher::SearchForTriangulation() which searches for geometrically reasonable feature matches using the
 * fundamental matrix relating the two keyframes. for each of the feature matches, first checks to make sure there is sufficient
 * parallax based on angle between viewing vectors in each keyframe and if there is triangulates the new point using
 * Triangulator::DirectLinearTriangulation() for monocular observations and backprojection for stereo-observations (which already have depth info).
 * Next puts the putative new landmark through a series of tests to ensure its quality:
 * 1) landmark must be in front of both keyframes
 * 2) reprojection error for landmark must be below a threshold in both keyframes
 * 3) distance from camera center to landmark must be reasonable (not zero)
 * 4) size of the observed features in each keyframe must be congruent, given distance of landmark from each keyframe
 * If the new point passes all these tests, a new landmark (mappoint) is created and associated with both keyframes.
 *
 */

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