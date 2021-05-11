#ifndef KEYFRAMECULLER_H_
#define KEYFRAMECULLER_H_

/*
 * a map job that removes redundant keyframe from a map
 * NOTE: uses keypoint octave - NEED TO ELIMINATE and substitute size.
 *
 * run() - main function, can be run in a separate thread (in principle, but need to do more checking
 * b/c at least one of the optional map jobs segfaults periodically when run concurrently)
 *   considers keyframe connected in covisibility graph to pKF_. for each of these local keyframes the function
 *   determines whether the mappoints viewed in the keyframe are redundantly viewed in other keyframes, ensuring the mappoints
 *   are viewed at the same scale (octave - NEED TO ELIMINATE) and are not distant points (which might be viewed by many different keyframe)
 *   if the fraction of redundant mappoints in a keyframe exceeds a threshold (as passed in KeyFrameCullerParameters) the
 *   KeyFrame is "removed" (SetBadKeyFrame).
 *
 */

#include <MapJob.h>
#include <MappingDataStructs.h>
#include <KeyFrame.h>
#include <Map.h>
#include <iostream>

namespace HYSLAM{
    class KeyFrameCuller : public MapJob {
    public:
        KeyFrameCuller(KeyFrame *pKF_, Map *pMap_, KeyFrameCullerParameters params_,  std::ofstream &log_);
        ~KeyFrameCuller();
        void run();

    private:
        KeyFrameCullerParameters params;
        KeyFrame* pKF;
        Map* pMap;
        std::ofstream* log;
    };

}//end namespace
#endif