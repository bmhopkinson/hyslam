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

#include "MapPoint.h"


namespace HYSLAM
{

long unsigned int MapPoint::nNextId=0;
std::mutex MapPoint::mGlobalMutex;
std::mutex MapPoint::mpt_creation_mutex;

MapPoint::MapPoint(const cv::Mat &Pos):
  //  mnFirstKFid(pRefKF->mnId), //mnFirstFrame(pRefKF->mnFrameId), 
    nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0),  mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(nullptr),  mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)
 //   ,mnFuseCandidateForKF(0)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpt_creation_mutex);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    std::unique_lock<std::mutex> lock2(mGlobalMutex);
    std::unique_lock<std::mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

void MapPoint::storeWorldPos(){
  mWorldPos_prev = mWorldPos;
}

int MapPoint::applyTransform(cv::Mat T){  //transfrom world position and normal of mappoint
  std::unique_lock<std::mutex> lock1(mMutexPos);

  //update world position
  storeWorldPos();
  cv::Mat pos_old = mWorldPos.clone();
  pos_old.push_back(cv::Mat::ones(1,1,CV_32F)); //make homogeneous
  cv::Mat pos_new = T* pos_old;
  pos_new.rowRange(0,3).copyTo(mWorldPos);

  //update normal vector;
  cv::Mat R = T.rowRange(0,3).colRange(0,3);
  mNormalVector = R * mNormalVector;  //validated this

  Thorn_applied = true;

  return 0;
}

cv::Mat MapPoint::GetNormal()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mpRefKF;
}


std::map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return nObs;
}

MapPoint* MapPoint::GetReplaced()
{
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    return mpReplaced;
}

bool MapPoint::isBad()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    return mbBad;
}

FeatureDescriptor MapPoint::GetDescriptor()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mDescriptor;
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

float MapPoint::GetMinDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

void MapPoint::SetProtection(int n, long unsigned int KFid)
{
    n_protected = n;
    protect_last_KFid = KFid;
}

void MapPoint::LowerProtection(unsigned long int KFid)
{
    if(protect_last_KFid != KFid){
        n_protected--;
        protect_last_KFid = KFid;
    }
}

} //namespace ORB_SLAM
