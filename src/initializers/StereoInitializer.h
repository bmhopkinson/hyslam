#ifndef STEREOINITIALIZER_H_
#define STEREOINITIALIZER_H_

/*
 * class for stereo initialization - derived from Initializer
 *
 * int initialize(Frame &frame) - attempt to initialize on frame, returns zero if successful; since points can be directly triangulated from stereo
 *  all this does is determine if there are sufficient features for initialization - currently doesn't ensure those points actually
 *  have stereomatches but it really should
 *
 * createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints) - creates and internal map from stored frame data.
 *  the successfully initialized frame is used to create a KeyFrame (pose set to origin) and keypoints with stereomatches are unprojected to create new mappoints
 *  associates mappoints with KeyFrame
 *  the new KeyFrame is returned in pKF1 (pKF2 = nullptr) and the new mappoints are returned in "mappoints"
 *
 *  transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T) - NOT IMPLEMENTED YET
 *
 *  Frame getInitializedFrame() - returns initialized frame stored in object, provides a warning if initialization isn't yet successful
 *
 *  addToMap(Map* pMap) - add KeyFrame and mappoints from internal, initial map to pMap.
 */

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
