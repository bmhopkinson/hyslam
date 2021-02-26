#ifndef STEREOINITIALIZER_H_
#define STEREOINITIALIZER_H_

#include <Initializer.h>
#include <InitializerDataStructs.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>

namespace HYSLAM{

class StereoInitializer : public Initializer {
    public:
        StereoInitializer(StereoInitializerParameters params_);
        int initialize(Frame &frame);
        int createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints);
        int transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T);
        Frame getInitializedFrame();
        KeyFrame* getCurrentKF();
        int addToMap(Map* pMap);
        void clear();
    
    private:
        StereoInitializerParameters params;
        //frames for attempted initialization
        Frame frame_init;
        bool init_success = false;

        //resulting initial map (two keyframes + mappoints)
        KeyFrame* pKFinit = nullptr;
        std::vector<MapPoint*> mpts;
        std::map<MapPoint*, int > mpts_to_keypts;
        bool map_created = false;
        double scale = 1.0000; // map scale

};

} //end namespace

#endif
