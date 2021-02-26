#ifndef KEYFRAMECULLER_H_
#define KEYFRAMECULLER_H_

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