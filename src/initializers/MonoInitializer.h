#ifndef MONOINITIALIZER_H_
#define MONOINITIALIZER_H_

/*
 * class for monocular initialization - derived from Initializer and makes heavy use of MonoEstimator
 * basic strategy - find matches between an initial frame and subsequent frames needing to have a high number of inlier matches
 * and sufficient baseline between frames for 3D triangulation. once this is achieved the two successful frames become new
 * KeyFrames and the triangulated points become an initial map.
 *
 * key functionality:
 * int initialize(Frame &frame) - pass frame to attempt to initialize on return zero if initialization in successful.
 *  internally, if there is no first reference frame the function firstFrame() is called which creates a new MonoInitializer.
 *  if there is a first reference frame then "frame" is instead passed to secondFrame(), which finds matches to features in the first
 *  frame using FeatureMatcher->SearchforInitialization and then passes feature matches into MonoInitializer to attempt to geometrically
 *  verify the matches and possibly triangulate the matches to 3D points in space. if initialization on second frame is successful
 *  the pose of the two frames are set (the first to the origin, the second based on the transform implied by the Homography or Fundamental matrix
 *  of the MonoInitializer)
 *
 * createMap() - creates the initial internal "map" which consists of: two KeyFrames constructed from the two initialized Frames,
 *  and mappoints created from the triangulated matches between those frames. associations are made between the KeyFrames and Mappoints,
 *  but no true "Map" data structure is created.
 *
 * createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints) - this is used to get the internal map data -
 *   references passed into function are populated with two initialized keyframes and mappoints from the internal map
 *
 * transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T) - transforms internal map so that it's orientation and scale are
 *  consistent with existing map - used when there's a SLAM camera for localization and imaging camera for the underlying system
 *
 * addToMap(Map* pMap) - adds internal map to existing map pMap
 */

#include <Initializer.h>
#include <InitializerDataStructs.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <Trajectory.h>
#include <MonoEstimator.h>
#include <Tracking_datastructs.h>
#include <FeatureFactory.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

namespace HYSLAM{

class MonoInitializer : public Initializer {
    public:
        MonoInitializer(MonoInitializerParameters params_, FeatureFactory* factory);
        int initialize(Frame &frame);
        bool hasValidFirstFrame();
        int firstFrame(Frame &frame); //, MonoInitialMatch &match_data);
        int secondFrame(Frame &frame); //, MonoInitialMatch &match_data);
        int createMap(); 
        int createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints);
        int transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T);
        int transformMapSE3(cv::Mat &Twc_SE3);
        int addToMap(Map* pMap);
        Frame getInitializedFrame();
        KeyFrame* getCurrentKF();
     //   int createInitialMap(Map* pMap_, Frame &F_ref, cv::Mat F_ref_T, Trajectory &trajectory, KeyFrame* &pKFini, KeyFrame* &pKFcur, Frame &mCurrentFrame  );
        void clear();
        
    private:
        MonoInitializerParameters params;
        MonoEstimatorParams estimator_params;
        FeatureFactory* feature_factory;

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
