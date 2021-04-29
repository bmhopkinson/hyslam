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

#ifndef MAPPOINT_H
#define MAPPOINT_H

/*
 *  class representing a LandMark (should probably change its name from mappoint to landmark)
 *  core data are:
 *      3D position
 *      Observations - Keyframes and associated Feature index where landmark has been observed
 *      Descriptor - representative feature descriptor used for matching with features in images
 *      size - radius of the LandMark (in world dimensions e.g. meters) - from mean of all observations
 *      normal - avg viewing direction between LandMark and KeyFrame centers
 *      max and min distance invariance  - range within which the landmark is considered valid for observing in an image
 *
 *  most the data is pushed down from MapPointDB - normal, observations, representative feature descriptor, size
 *  the only functionality is:
 *  applyTransform(cv::Mat T) which applies the transform T (similarity or SE3) to position and normal
 *
 *  mappoints can be protected from culling by SetProtection()
 *
 */

#include <FeatureDescriptor.h>

#include <opencv2/core/core.hpp>
#include <mutex>
#include <map>

namespace HYSLAM
{

class KeyFrame;


class MapPoint
{
public:
 //   MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF);
    MapPoint(const cv::Mat &Pos);


    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();
    void storeWorldPos();
    
    int applyTransform(cv::Mat T);
    
    KeyFrame* GetReferenceKeyFrame();
    void setReferenceKeyFrame(KeyFrame* pKF){ mpRefKF = pKF;  }

    cv::Mat GetNormal();
    void setNormal(const cv::Mat norm) {mNormalVector = norm.clone();}

    std::map<KeyFrame*,size_t> GetObservations();
    void setObservations(const std::map<KeyFrame*,size_t> &obs) {mObservations = obs;}
    void setNObs(int n){nObs = n; }
    int Observations();

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    FeatureDescriptor GetDescriptor();
    void setDescriptor(const FeatureDescriptor desc){mDescriptor = desc;}
   // std::map<KeyFrame*, cv::Mat> GetAllDescriptors();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    void setMinDistanceInvariance(float mind){mfMinDistance = mind;}
    void setMaxDistanceInvariance(float maxd){mfMaxDistance = maxd;}

    bool Protected() {return n_protected >0; };
    void SetProtection(int n, unsigned long int KFid);
    void LowerProtection(unsigned long int KFid);

    float getSize() const;
    void setSize(float size);
    float getMeanDistance() const;
    void setMeanDistance(float meanDistance);

    // void Replace(MapPoint* pMP);
    void setReplaced(MapPoint* pMP_new){mpReplaced = pMP_new;}
    MapPoint* GetReplaced();

    void setBad(){ mbBad = true; };
    bool isBad();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
 //   long int mnFirstFrame;


    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
 //   long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    //variables used by imaging BA
    bool Thorn_applied = false;

    static std::mutex mGlobalMutex;

protected:

    static std::mutex mpt_creation_mutex;

     // Position in absolute coordinates
     cv::Mat mWorldPos;
     cv::Mat mWorldPos_prev;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     float size; //mean size at mean distance
     float mean_distance;

     int nObs;
     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to enable fast matching
     FeatureDescriptor mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;

     int n_protected = 0;
     long unsigned int protect_last_KFid = 0;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
