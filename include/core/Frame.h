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

#ifndef FRAME_H
#define FRAME_H

#include<vector>
#include<string>

#include <MapPoint.h>
#include <Camera.h>

#include <ORBVocabulary.h>
#include <FeatureViews.h>
#include <src/main/ORBSLAM_datastructs.h>
#include <LandMarkMatches.h>
#include <SensorData.h>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <mutex>


namespace HYSLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

//class MapPoint;
class KeyFrame;
//class LandMarkMatches;

//NEED TO BE ABLE TO QUERY FRAME w/ a mappoint and find out if it's matched to this frame and if so obtain the associated keypoint data

class Frame
{
public:
    Frame();
    // Constructor
    Frame( const double &timeStamp, ORBViews views_, ORBVocabulary* voc, const Camera &camdata,
                  const std::string img_name, const  SensorData sensor_d, bool stereo);

    // Copy constructor.
    Frame(const Frame &frame);

    //assignment operator - caused problems w/ opencv mats
    Frame& operator=(const Frame&);

    bool isEmpty(){return is_empty;} const //is frame empty;
    bool isTracked(){return is_tracked;} const //was frame tracked correctly in Tracking
    void setTracked(bool tracked) {is_tracked = tracked;}

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){return mOw.clone();}

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse() {return mRwc.clone();}

    //project landmark (mappoint) into frame, return whether landmark is visible, pass image coords in uv_ur vector (uv in left image, ur in right image if stereo,  mono -1)
    bool ProjectLandMark(MapPoint* pMP, cv::Mat &uv_ur);
    bool ProjectLandMark(cv::Mat P, cv::Mat &uv_ur);
    float ReprojectionError(cv::Mat P, int idx); //reprojection error of 3D point P relative to landmarkview of idx

    //determine if landmark i is assocaited with a mappoint, if so return it, otherwise return nullptr
    MapPoint* hasAssociation(int i) const;
    int hasAssociation(MapPoint* pMP) const;

    //NEED TO MAKE FUNCTIONS THAT CHANGE ASSOCIATIONS THREAD SAFE
    //associate Landmark i with mappoint pMP; if "replace" allows replacement of previously matched maptpoint, return 0 if succesful
    int associateLandMark(int i, MapPoint* pMP, bool replace);
    int associateLandMarks(std::map<size_t, MapPoint*> associations, bool replace);

    //as above, but assumes vector length = N keypoints and each entry is an assocaited mappoint (or nullptr),
    int associateLandMarkVector(std::vector<MapPoint*> vpMapPointMatches, bool replace);

    // remove mappoint association with KeyPoint i.
    int removeLandMarkAssociation(int i);

    int getAssociatedLandMarks(std::vector<cv::KeyPoint> &features, std::vector<MapPoint*> &landmarks );

    void propagateTracking(Frame &frame_previous);

    //clear all mappoint to keypoint associations
    int clearAssociations();
    int validateNewAssociations(std::vector<MapPoint*> mvpMapPoints) const;//debugging
    
    int predictScale(const float &currentDist, MapPoint* pMP);

    bool isOutlier(int i) const ;
    int  setOutlier(int i, bool is_outlier);

    std::vector<MapPoint*> replicatemvpMapPoints() const; // termporary for eliminating mvpMapPoints from Frame 

    // determine rotation quat relative to first KeyFrame
    void CalcRelativeQuat();

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

   //getters
    const Camera& getCamera() const {return camera;}
    const ORBViews& getViews() const {return views; }
    ORBViews copyViews() const {return views;}
    const LandMarkMatches& getLandMarkMatches() {return matches;}
    LandMarkMatches copyLandMarkMatches() {return matches;}
    SensorData getSensorData(){return sensor_data;}

    //tests
    void validateMatches();

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    //Camera data
    Camera camera; //mCamdata

    // image name associated with frame
    std::string fimgName;

    float mThDepth;    // Threshold close/far points. Close points are inserted from 1 view. // Far points are inserted as in the monocular case from 2 views.

    // Frame timestamp.
    double mTimeStamp;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    int N; // Number of KeyPoints.
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF = nullptr;

    // Undistorted Image Bounds
    float mnMinX;
    float mnMaxX;
    float mnMinY;
    float mnMaxY;

private:
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc

    bool is_stereo;
    bool is_empty;
    bool is_tracked = false;

    ORBViews views;  //landmark views (features)
    LandMarkMatches matches; //data association matches between landmarks and landmark views

    SensorData sensor_data;
};

}// namespace ORB_SLAM

#endif // FRAME_H
