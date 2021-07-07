/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAME_H
#define KEYFRAME_H

/*
 * represents an "important" single image (mono or stereo), its pose in the world, its features, and landmark matches - used primarily in MAPPING
 * KeyFrames are persistent and stored in maps, they are constructed from a Frame from which they inherit much of their initial data
 * functionality - handles projecting points into frame (and unprojecting), handles associating/disassociating landmarks to feature views- this should NOT be done directly, associations should generally be made through a Map
 *              also divides image into a grid and assigns keypoint to grid elements for rapid retrival of keypoint in a region for feature matching to landmarks
 *              landmark matches are local to this Frame, if the frame is converted to a KeyFrame such associations become part of covisiblity graph, etc
 * NEEDS some work: would like to derive Frame and KeyFrame from a common base "Frame" class. should not refer to  Frame, and overall its a bit of a mess - lots of legacy garbage that should be cleaned up
 *
 * key functions:
 * ComputeBoW - compute bag of word vector for this frame - time consuming so it's only done when needed
 *    bool ProjectLandMark(MapPoint* pMP, cv::Mat &uv_ur) - determines uv (and ur for stereo) pixel coordinates of 3D position of landmark pMP- returns true if projects to point visible in frame
 *    bool ProjectLandMark(cv::Mat P, cv::Mat &uv_ur) - determines uv (and ur for stereo) pixel coordinates of 3D position P - returns true if projects to point visible in frame
 *    float ReprojectionError(cv::Mat P, int idx); //reprojection error of 3D point P relative to landmarkview of idx
 *    int associateLandMark(int i, MapPoint* pMP, bool replace); associates keypoint i with landmark pMP, if "replace" is true will overwrite association to i if one already exists - should be called through a Map
 *    std::set<KeyFrame *> GetConnectedKeyFrames(); returns all connected keyframes - data for this is pushed down from Map/Covis Graph
 *    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N); returns N best covisible keyframes based on number of shared landmarks - data for this is pushed down from Map/Covis Graph
 *    spanning tree info pushed down from Map/Spanning tree:     KeyFrame* GetParent(); void setParent(KeyFrame* pKF);
 *
 *
 */

#include <MapPoint.h>
#include <FeatureViews.h>
#include <Frame.h>
#include <Camera.h>
#include <LandMarkMatches.h>
#include <FeatureVocabulary.h>
#include <FeatureViews.h>
#include <SensorData.h>


#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <set>
#include <vector>


namespace HYSLAM
{

class Frame;  //remove eventually - needed due to cyclic dependency between Frame/KeyFrame
class Map;  //intentional  - getMap() returns ref to dumb pointer. 

class KeyFrame
{
public:
    KeyFrame(Frame &F);//, Map* pMap_);//, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat getCameraMatrix();
    void storePose();

    bool ProjectLandMark(MapPoint* pMP, cv::Mat &uv_ur);  //project landmark (mappoint) into frame, return whether landmark is visible, pass image coords in uv_ur vector (uv in left image, ur in right image if stereo,  mono -1)
    bool ProjectLandMark(cv::Mat P, cv::Mat &uv_ur);
    float ReprojectionError(cv::Mat P, int idx); //reprojection error of 3D point P relative to landmarkview of idx
    bool isLandMarkVisible(MapPoint * pMP);


    MapPoint* hasAssociation(int i) const;   //determine if landmarkview i is assocaited with a mappoint, if so return it, otherwise return nullptr
    int hasAssociation(MapPoint* pMP) const; //determine if landmark pMP is assocaited with a landmarkview, if so return it, otherwise return -1

    int associateLandMark(int i, MapPoint* pMP, bool replace);     //associate Landmark i with mappoint pMP; if "replace" allows replacement of previously matched maptpoint, return 0 if succesful
    int associateLandMark(int i, MapPoint* pMP, bool replace, MapPoint* &pMP_old);     //associate Landmark i with mappoint pMP; if "replace" allows replacement of previously matched maptpoint, return 0 if succesful
    int associateLandMarkVector(std::vector<MapPoint*> vpMapPointMatches, bool replace); //as above, but assumes vector length = N keypoints and each entry is an assocaited mappoint (or nullptr),
    int removeLandMarkAssociation(int i); // remove mappoint association with KeyPoint i.
    int removeLandMarkAssociation(MapPoint* pMP);
    int getAssociatedLandMarks(std::vector<cv::KeyPoint> &features, std::vector<MapPoint*> &landmarks );
    std::vector<MapPoint*>  getAssociatedLandMarks(); // return vector of valid associated landmarks
    std::set<MapPoint*> GetMapPoints();
    int TrackedMapPoints(const int &minObs);
    bool isMapPointMatched(MapPoint* pMP);
  //  int predictScale(const float &currentDist, MapPoint* pMP);
    float featureSizeMetric(int idx); // size of feature
    float landMarkSizePixels(MapPoint* lm);

    int clearAssociations(); //clear all mappoint to keypoint associations


    //sensor data
    void setSensorData(SensorData s);
    void SetRefGPS(SensorData s);
    void SetRefQuat(SensorData s);
    static  std::vector<double> GetRefQuat();
    const SensorData& getSensorData() const {return sensor_data; }
    SensorData copySensorData() const {return sensor_data;}

    // Bag of Words Representation
    void ComputeBoW();

    // Corresponding Frame or Image name
    std::string kfImgName;

    // Covisibility graph functions - data set by KeyFrameDB
    void setCovisibleKeyFrames(std::vector<KeyFrame*> & KFs_covis){mvpOrderedConnectedKeyFrames = KFs_covis;};
    std::set<KeyFrame *> GetConnectedKeyFrames(); //used in LoopClosing and KeyFrameDatabase
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N); //used in Tracking, KeyFrameDatabase, LocalMapping

    //spanning graph parent info - data set by KeyFrameDB
    KeyFrame* GetParent(); // used in Tracking, Optimizer, System, Trajectory, LocalMapping, MapDrawer - used whenever a KeyFrame is bad to work back tree
    void setParent(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

 //   std::vector<MapPoint*> replicatemvpMapPoints() const; // termporary for eliminating mvpMapPoints from KEyFrame
 //   int validateNewAssociations();
   
    bool isOutlier(int i) const ;
    int  setOutlier(int i, bool is_outlier);

    const LandMarkMatches& getLandMarkMatches() {return matches;}

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int idx);
    cv::Mat BackProjectLandMarkView(int idx, float depth);

    // Image, projection
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable/test keyframe flags
    void setProtection();
    void clearProtection();
    bool isProtected();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    //getters
    bool isStereo() const {return is_stereo; }
    const Camera& getCamera() const {return camera;}
    const FeatureViews& getViews() const {return views; }
    FeatureViews copyViews() const {return views;}
    //  void setMap(Map* pMap_){pMap = pMap_;}
  //  Map* getMap(){return pMap;}

    std::vector<MapPoint*> GetMapPointMatches();
    std::vector<MapPoint*> replicatemvpMapPoints() const; // termporary for eliminating mvpMapPoints from KEyFrame

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    //Camera data - ultimately consolidate all camera info into this struct
    Camera camera;
    float mThDepth;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
   // long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    //variable used by imaging BA
    cv::Mat Tcw_prev;
    bool Thorn_applied = false;

    // Number of KeyPoints
    const int N;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    // Bad flags
   // bool mbNotErase;

    bool mbToBeErased;
    bool mbBad;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // Calibration parameters - can't eliminate these from the header without impacting tracking performance (but program will compile w/out them - very mysterious!)
    float fx, fy, cx, cy, mbf;// invfx, invfy,  mb;
    cv::Mat mK;
    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middle point. Only for visualization

    //flags
    bool mbProtected;

    // sensor data
    SensorData sensor_data;
	static std::vector<double> RefQuat;  //reference quaternion of first keyframe - used to calculate orientation of camera from IMU data relative to first keyframe
    static std::vector<double> RefGPS; //reference GPS position of first keyframe - origin for relative GPS positions

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    FeatureVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;

    // Spanning Tree and Loop Edges
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspLoopEdges;

    float mHalfBaseline; // Only for visualization

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    static std::mutex class_mutex;

    bool is_stereo;

    // feature data
    FeatureViews views;  //landmark views (features)
    LandMarkMatches matches;  //data association matches between landmarks and landmark views
    Map* pMap;
    
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
