#ifndef MONOINITIALIZER_H_
#define MONOINITIALIZER_H_

#include <Initializer.h>
#include <InitializerDataStructs.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>
#include <MonoEstimator.h>
#include <Tracking_datastructs.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

namespace ORB_SLAM2{

// creates an initial map (consisting of two keyframes and associated mappoints) from two succesfully initialized frames 
// can scale the initial map if desired. 

/*
 *logic should go:
 * firstFrame()
 * secondFrame()
 * if success createMap() - internally creates map of two KFs and mappoint - not associated with existing map
 * optional transformMap(Trajectory, Tcam ,etc)
 * addToMap(Map*) - adds internal map to existing map
 * mCurrentframe = getFinalizedSecondFrame();  - need to revise copy constructor. 
 * 
 */
 
class MonoInitializer : public Initializer {
    public:
        MonoInitializer(MonoInitializerParameters params_);
        int initialize(Frame &frame);
        bool hasValidFirstFrame();
        int firstFrame(Frame &frame); //, MonoInitialMatch &match_data);
        int secondFrame(Frame &frame); //, MonoInitialMatch &match_data);
        int createMap(); 
        int createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints);
        int transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T);
        int addToMap(Map* pMap);
        Frame getInitializedFrame();
        KeyFrame* getCurrentKF();
     //   int createInitialMap(Map* pMap_, Frame &F_ref, cv::Mat F_ref_T, Trajectory &trajectory, KeyFrame* &pKFini, KeyFrame* &pKFcur, Frame &mCurrentFrame  );
        void clear();
        
    private:
        MonoInitializerParameters params;
        MonoEstimatorParams estimator_params;

        //frames for attempted initialization
        Frame first_frame;
        Frame second_frame;
        bool has_data = false; //inidicates whether first frame has been set
        bool init_success = false;
        
        //resulting initial map (two keyframes + mappoints)
        KeyFrame* pKFfirst;
        KeyFrame* pKFsecond;
        std::vector<MapPoint*> mpts; 
        bool map_created = false;
        double scale = 1.0000; // map scale 
        std::map<MapPoint*, std::pair<int, int> > mpts_to_keypts;  
        
       // Map* pMap;  //needed for keyframe construction
       // std::string cam_type;

        
        //match data between frames
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        
        MonoEstimator* mpInitializer = nullptr;

};

}//end namespace

#endif
