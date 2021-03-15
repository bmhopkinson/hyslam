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

#include <KeyFrame.h>
#include <Converter.h>
#include <ORBSLAM_datastructs.h>
#include <iostream>



namespace HYSLAM
{

std::vector<double> KeyFrame::RefQuat = {-999.9, -999.9, -999.9,-999.9};
std::vector<double> KeyFrame::RefGPS = {-999.9, -999.9, -999.9};
long unsigned int KeyFrame::nNextId=0;
std::mutex KeyFrame::class_mutex;

KeyFrame::KeyFrame(Frame &F) ://, Map* pMap_):
    kfImgName(F.fimgName), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), camera(F.camera),
    mThDepth(F.mThDepth), N(F.N), mBowVec(F.mBowVec), mFeatVec(F.mFeatVec),
    mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mpORBvocabulary(F.mpORBvocabulary), pMap(nullptr), mpParent(NULL), mbProtected(false),mbBad(false), mbToBeErased(false)
   //, mnFuseTargetForKF(0)
    
{
    {
        std::lock_guard<std::mutex> lock(class_mutex);  //currently keyframes only created in tracking thread so this isn't strictly necesary but this could be helpful in future
        mnId = nNextId++;
    }
   // std::cout << "in keyframe constructor: " << mnId << std::endl;

    views = F.copyViews();
    matches = F.copyLandMarkMatches();

    mHalfBaseline = camera.mb()/2;


    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
    setSensorData(F.getSensorData());

    mvpMapPoints = F.replicatemvpMapPoints();

   // ComputeBoW();
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
     //   std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(views.getDescriptors());
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        std::vector<FeatureDescriptor> descriptors =  views.getDescriptors();
        mpORBvocabulary->transform(descriptors,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::getCameraMatrix(){
    cv::Mat P = camera.K * Tcw.rowRange(0,3);
    return P.clone();
}
/*
Eigen::Quaterniond KeyFrame::GetQuat()
{
    if(imud.valid){
       Eigen::Quaterniond quatRel  = Converter::toQuatEigen(imud.quat);
	   return quatRel; // currently this is data that will not be updated so no need for mutex protection
    }
    else {
       std::cout << "no valid imu data, cannot KeyFrame::GetQuat()" << std::endl;
    }
}
*/
void KeyFrame::storePose(){
  Tcw_prev = Tcw.clone();
}


void KeyFrame::setSensorData(SensorData s){
    GpsData gdata = s.getGPS();
    gdata.origin = RefGPS;
    gdata.relpos = {gdata.noreast.first - RefGPS[0], gdata.noreast.second - RefGPS[1], gdata.alt - RefGPS[2] };
    s.setGPS(gdata);
    sensor_data = s;
}

std::vector<double> KeyFrame::GetRefQuat(){
	return RefQuat;
}

void KeyFrame::SetRefQuat(SensorData s){
	RefQuat =  s.getQuat();
}

void  KeyFrame::SetRefGPS(SensorData s){
    GpsData gd = s.getGPS();
    RefGPS = {gd.noreast.first, gd.noreast.second, gd.alt };
    setSensorData(sensor_data);
}

std::set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return std::set<KeyFrame*>( mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.end());
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return std::vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

std::set<MapPoint*> KeyFrame::GetMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::set<MapPoint*> s;
    for(auto it = matches.cbegin(); it != matches.cend(); ++it){
        MapPoint* pMP = it->second;
        if(!pMP){continue;}
        if(!pMP->isBad()){
            s.insert(pMP);
        }
    }
    return s;

}

int KeyFrame::TrackedMapPoints(const int &minObs) {
    std::unique_lock<std::mutex> lock(mMutexFeatures);

    int nPoints = 0;
    const bool bCheckObs = minObs > 0;

    for (auto it = matches.cbegin(); it != matches.cend(); ++it) {
        MapPoint *pMP = it->second;
        if(!pMP){continue;}
        if (!pMP->isBad()) {
            if(bCheckObs) {
                if (pMP->Observations() >= minObs) {
                    ++nPoints;
                }
            } else {
                ++nPoints;
            }
        }
    }
    return nPoints;

}

bool KeyFrame::isMapPointMatched(MapPoint* pMP){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
  int keypt = matches.hasAssociation(pMP);
  if(keypt < 0 ){
      return false;
  } else {
      return true;
  }
}

int KeyFrame::predictScale(const float &currentDist, MapPoint* pMP){

    float ratio = pMP->GetMaxDistanceInvariance()/(1.2*currentDist);  //(1.2 factor is oddity from GetMaxDistanceInvariance())
    int nScale_this = ceil(log(ratio)/views.orbParams().mfLogScaleFactor);
    int nScales = views.orbParams().mnScaleLevels;
    if(nScale_this <0)
        nScale_this = 0;
    else if(nScale_this >=nScales)
        nScale_this = nScales-1;

    return nScale_this;
}

KeyFrame* KeyFrame::GetParent()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mpParent;
}

void KeyFrame::setParent(KeyFrame* pKF){
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mpParent = pKF;
}
    

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    setProtection();
    mspLoopEdges.insert(pKF);
}

std::set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::setProtection()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    mbProtected = true;
}

void KeyFrame::clearProtection()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    mbProtected = false;
}

bool  KeyFrame::isProtected(){
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mbProtected;
}

bool KeyFrame::isBad()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mbBad;
}

std::vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = std::min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = std::min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                //const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const cv::KeyPoint kpUn = views.keypt(vCell[j]);
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    //const float z = mvDepth[i];
    const float z = views.depth(i);
    if(z>0)
    {
        const float u = views.keypt(i).pt.x;
        const float v = views.keypt(i).pt.y;
        cv::Mat x3Dc = camera.Unproject(u,v, z);

        std::unique_lock<std::mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc + Ow;
    }
    else
        return cv::Mat();

}

cv::Mat KeyFrame::BackProjectLandMarkView(int idx, float depth){
    const float u = views.keypt(idx).pt.x;
    const float v = views.keypt(idx).pt.y;
    cv::Mat x3Dc = camera.Unproject(u,v, depth);

    std::unique_lock<std::mutex> lock(mMutexPose);
    return Twc.rowRange(0,3).colRange(0,3)*x3Dc + Ow;
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    std::vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        std::unique_lock<std::mutex> lock2(mMutexPose);
        Tcw_ = Tcw.clone();
    }

    std::set<MapPoint*>  spMP;
    spMP = GetMapPoints();

    std::vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(auto it = spMP.begin(); it != spMP.end(); ++it){
        MapPoint* pMP = *it;
        if(!pMP){continue;}
        if(!pMP->isBad()) {
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }

    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

MapPoint* KeyFrame::hasAssociation(int i) const{
    return matches.hasAssociation(i);
}

int KeyFrame::hasAssociation(MapPoint* pMP) const{
    return matches.hasAssociation(pMP);
}

int KeyFrame::associateLandMark(int i, MapPoint* pMP, bool replace)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return matches.associateLandMark(i, pMP, replace);
}

int KeyFrame::associateLandMarkVector(std::vector<MapPoint*> vpMapPointMatches, bool replace){
   int vec_size = vpMapPointMatches.size();
   if(vec_size != N){
    return -1;
   }

    std::unique_lock<std::mutex> lock(mMutexFeatures);
   int errors = 0;
   for(int i = 0; i < vec_size; i++){
       MapPoint* pMP = vpMapPointMatches[i];
       if(pMP){
          errors+= matches.associateLandMark(i, pMP, replace);
       }
   }
   if(errors == 0){
     return 0;
     std::cout << "succesfully associated mappoint vector" <<std::endl;
   }
   else{
     return -1;
   }
}


// remove mappoint association with KeyPoint i.
int KeyFrame::removeLandMarkAssociation(int i){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return matches.removeLandMarkAssociation(i);
}

int KeyFrame::removeLandMarkAssociation(MapPoint* pMP){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return matches.removeLandMarkAssociation(pMP);
}


int KeyFrame::getAssociatedLandMarks(std::vector<cv::KeyPoint> &features, std::vector<MapPoint*> &landmarks ){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    for(auto it = matches.cbegin(); it != matches.cend(); ++it){
        int LMid = it->first;
        cv::KeyPoint keypt = views.keypt(LMid);
        features.push_back(keypt);
        landmarks.push_back(it->second);
    }
    return 0;
}

 std::vector<MapPoint*>  KeyFrame::getAssociatedLandMarks(){
    std::vector<MapPoint*> landmarks;
     std::unique_lock<std::mutex> lock(mMutexFeatures);
    for(auto it = matches.cbegin(); it != matches.cend(); ++it){
        MapPoint* lm = it->second;
        if(!lm){continue;}
        if(!lm->isBad()) {
            landmarks.push_back(lm);
        }
    }
    return landmarks;
}

//clear all mappoint to keypoint associations
int KeyFrame::clearAssociations(){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return matches.clearAssociations();
}

bool KeyFrame::isOutlier(int i) const {
    return matches.isOutlier(i);
}
int  KeyFrame::setOutlier(int i, bool is_outlier){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return matches.setOutlier(i, is_outlier);
}


bool KeyFrame::ProjectLandMark(MapPoint* pMP, cv::Mat &uv_ur){
  cv::Mat P = pMP->GetWorldPos();
  return ProjectLandMark(P, uv_ur);

}

bool KeyFrame::ProjectLandMark(cv::Mat P, cv::Mat &uv_ur){

  // 3D in camera coordinates
  const cv::Mat Pc = Tcw.rowRange(0,3).colRange(0,3)*P+Tcw.rowRange(0,3).col(3);  // Rcw*P+tcw;
  return camera.Project(Pc, uv_ur);

}

float KeyFrame::ReprojectionError(cv::Mat P, int idx){

    cv::Mat uv_ur;
    if(ProjectLandMark(P, uv_ur)){ //does landmark position even project into the frame
       cv::KeyPoint kpt = views.keypt(idx);
       float u = uv_ur.at<float>(0,0);
       float v = uv_ur.at<float>(1,0);
       float errX =  u - kpt.pt.x;
       float errY =  v - kpt.pt.y;

       float errXr = 0.000;

       float ur_view = views.uR(idx);
       if(ur_view >= 0.0){
           float ur = uv_ur.at<float>(2,0);
           errXr = ur - ur_view ;
       }

        float sserr = errX*errX + errY*errY + errXr*errXr;
        return sserr;

    } else {
        return 10000.00; //large value - alternatively return negative number to indicate it can't be projected
    }
}

bool  KeyFrame::isLandMarkVisible(MapPoint * pMP){
   cv::Mat dummy;
   return ProjectLandMark(pMP, dummy);
}


std::vector<MapPoint*>  KeyFrame::replicatemvpMapPoints() const
{
    std::vector<MapPoint*> mvp_sim;
    for(int i = 0; i < N; ++i){
        mvp_sim.push_back(matches.hasAssociation(i));
    }
    return mvp_sim;
}


std::vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return replicatemvpMapPoints();
}



} //namespace ORB_SLAM
