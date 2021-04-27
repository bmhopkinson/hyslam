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

#include "Frame.h"
#include <KeyFrame.h>
#include "Converter.h"
#include <thread>

namespace HYSLAM
{

long unsigned int Frame::nNextId=0;

Frame::Frame() : is_empty(true)
{
  //  std::cout << "in default frame constructor, is_empty:" << isEmpty() << std::endl;
}

//constructor

Frame::Frame(const double &timeStamp, FeatureViews views_, FeatureVocabulary* voc,
             const Camera &camdata, const std::string img_name, const  SensorData sensor_d, bool stereo)
    : mTimeStamp(timeStamp), feature_vocabulary(voc), camera(camdata), fimgName(img_name), sensor_data(sensor_d),
      mpReferenceKF(static_cast<KeyFrame*>(NULL)), views(views_), is_stereo(stereo), is_empty(false), is_tracked(false)
{
    // Frame ID
    mnId=nNextId++;
    mThDepth = camdata.thDepth;

    N = views.numViews();


    if(N==0)
        return;

  //  UndistortKeyPoints();
    mnMinX = camdata.mnMinX;
    mnMaxX = camdata.mnMaxX;
    mnMinY = camdata.mnMinY;
    mnMaxY = camdata.mnMaxY;

    mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

    AssignFeaturesToGrid();
    CalcRelativeQuat();

}

//Copy Constructor
Frame::Frame(const Frame &frame)
    : feature_vocabulary(frame.feature_vocabulary), camera(frame.camera), fimgName(frame.fimgName),
      mThDepth(frame.mThDepth), mTimeStamp(frame.mTimeStamp), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
      N(frame.N), mfGridElementWidthInv(frame.mfGridElementWidthInv), mfGridElementHeightInv(frame.mfGridElementHeightInv),
      mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnMinX(frame.mnMinX), mnMaxX(frame.mnMaxX), mnMinY(frame.mnMinY), mnMaxY(frame.mnMaxY),
      is_stereo(frame.is_stereo), views(frame.views), matches(frame.matches), sensor_data(frame.sensor_data), is_empty(frame.is_empty),
      is_tracked(frame.is_tracked)
{
//    std::cout << "in frame copy const" <<std::endl;
    for(int i=0;i<FRAME_GRID_COLS;i++){
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
        }
    }

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame& Frame::operator=(const Frame& frame){
    if(this == &frame)
        return *this;

    is_empty = frame.is_empty;
    feature_vocabulary = frame.feature_vocabulary;
    camera = frame.camera;
    fimgName = frame.fimgName;
    mThDepth = frame.mThDepth;
    mTimeStamp = frame.mTimeStamp;
    mBowVec = frame.mBowVec;
    mFeatVec = frame.mFeatVec;
    N = frame.N;
    mfGridElementWidthInv = frame.mfGridElementWidthInv ;
    mfGridElementHeightInv = frame.mfGridElementHeightInv;
    
    for(int i=0;i<FRAME_GRID_COLS;i++){
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
        }
    }
    mTcw= frame.mTcw.clone();
    mnId = frame.mnId;
    mpReferenceKF = frame.mpReferenceKF;
    mnMinX = frame.mnMinX;
    mnMaxX = frame.mnMaxX;
    mnMinY = frame.mnMinY;
    mnMaxY = frame.mnMaxY;
    
    if(!frame.mTcw.empty()){
        SetPose(frame.mTcw);
    }

    is_stereo = frame.is_stereo;
    is_tracked = frame.is_tracked;
    
    views = frame.views;
    matches = frame.matches;
    sensor_data = frame.sensor_data;
    
    return *this;
    
}

void Frame::AssignFeaturesToGrid()
{
    std::vector<cv::KeyPoint> keypts = views.getKeys();
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = keypts[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

}

bool Frame::ProjectLandMark(MapPoint* pMP, cv::Mat &uv_ur){
  // 3D in absolute coordinates
  cv::Mat P = pMP->GetWorldPos();
  //std::cout << "world pos: " << P << std::endl;
  return ProjectLandMark(P, uv_ur);
}

bool Frame::ProjectLandMark(cv::Mat P, cv::Mat &uv_ur){
    const cv::Mat Pc = mRcw*P+mtcw; //rotation into camera coordinates
   // std::cout << "camera pos: " << Pc << std::endl;
    bool valid =  camera.Project(Pc, uv_ur);
  //  std::cout << "frame projected pos: u,v " << uv_ur.at<float>(0) << "\t"  << uv_ur.at<float>(1) << std::endl;
    return valid;
}
float Frame::ReprojectionError(cv::Mat P, int idx){

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

MapPoint* Frame::hasAssociation(int i) const{
    return matches.hasAssociation(i);
}

int Frame::hasAssociation(MapPoint* pMP) const{
    return matches.hasAssociation(pMP);
}


int Frame::associateLandMark(int i, MapPoint* pMP, bool replace)
{
  return matches.associateLandMark(i, pMP, replace);

//below is old code for a bidirectional map (two maps) but this structure is not appropriate b/c multiple keypoints can be associated w/ the same mappoint
/*   int idx_old = hasAssociation(pMP);
   std::cout << "mpt_old: " << mpt_old << ", idx_old: " << idx_old;
   if( (mpt_old != nullptr) || (idx_old >= 0 )){
     if(replace){
       std::cout << "   replacing  " <<std::endl;
       views_to_mpts[i] = pMP;
       mpts_to_views[pMP] = i;

       //erase dangling associations.
       if(mpt_old!=nullptr){
         if( mpt_old != pMP){  //if replacing index but not mappoint don't erase mappoint
          mpts_to_views.erase(mpt_old);
         }
       }

       if(idx_old >= 0){
         if(idx_old != i){   //if replacing  mpt but not index - don't erase index
           views_to_mpts.erase(idx_old);
         }
       }

       return 0;
     }
     else {  //not allowed to replacereplacing
       return -1;
     }
   } else {
            std::cout << "   inserting  " <<std::endl;
     views_to_mpts.insert({i, pMP});
     mpts_to_views.insert({pMP, i});
     return 0;
   }
*/
}
int  Frame::associateLandMarks(std::map<size_t, MapPoint*> associations, bool replace){
    int n_associated;
    for(auto it = associations.begin(); it != associations.end(); ++it){
        size_t idx = it->first;
        MapPoint* lm = it->second;
        int res = matches.associateLandMark(idx, lm, replace);
        if(res == 0){
            n_associated++;
        }
    }
    return n_associated;
}

int Frame::associateLandMarkVector(std::vector<MapPoint*> vpMapPointMatches, bool replace){
   int vec_size = vpMapPointMatches.size();
   if(vec_size != N){
    return -1;
   }

   int errors = 0;
   for(int i = 0; i < vec_size; i++){
       MapPoint* pMP = vpMapPointMatches[i];
       if(pMP){
          errors+= matches.associateLandMark(i, pMP, replace);
       }
   }
   if(errors == 0){
     return 0;
    // std::cout << "succesfully associated mappoint vector" <<std::endl;
   }
   else{
     return -1;
   }
}


// remove mappoint association with KeyPoint i.
int Frame::removeLandMarkAssociation(int i){
  return matches.removeLandMarkAssociation(i);
}

int Frame::getAssociatedLandMarks(std::vector<cv::KeyPoint> &features, std::vector<MapPoint*> &landmarks ){
   // std::vector<MapPoint*> tempvec;
    for(auto it = matches.cbegin(); it != matches.cend(); ++it){
        int LMid = it->first;
        cv::KeyPoint keypt = views.keypt(LMid);
        features.push_back(keypt);
        landmarks.push_back(it->second);
    }
  //  landmarks = tempvec;
    return 0;
}

float Frame::featureSizeMetric(int idx){ //consider each feature to be a square associated with a mappoint  normal to camera line of sight
    MapPoint* lm = hasAssociation(idx);
    if(!lm){
        return -1.0;
    }

    cv::Mat Pos = lm->GetWorldPos();
    cv::Mat Ow = GetCameraCenter();
    cv::Mat Pos_cam = Pos - Ow;
    float z  = cv::norm(Pos_cam);
    if(z < 0.0){ return -1.0;}

    const float u = views.keypt(idx).pt.x;
    const float v = views.keypt(idx).pt.y;
    float radius = views.keypt(idx).size/2;
    cv::Mat left_edge = camera.Unproject(u-radius,v, z);  //could be simplified w/ pinhole camera but wanted to keep more general
    cv::Mat right_edge = camera.Unproject(u+radius,v, z);
    cv::Mat length = right_edge - left_edge;
    float size = cv::norm(length);

    return size;

}

float Frame::landMarkSizePixels(MapPoint* lm){
    int idx = hasAssociation(lm);
    if(idx >= 0){  //if already associated just use observed keypt size
        std::cout <<"idx: " << idx <<  " ,using old size: " << views.keypt(idx).size << std::endl;
        return views.keypt(idx).size;
    } else { //project it
        cv::Mat Pw = lm->GetWorldPos();
        cv::Mat Pw_left_edge = Pw.clone();
        Pw_left_edge.at<float>(0,0) = Pw_left_edge.at<float>(0,0) - lm->getSize()/2;
        cv::Mat Pw_right_edge = Pw.clone();
        Pw_right_edge.at<float>(0,0) = Pw_right_edge.at<float>(0,0) + lm->getSize()/2;
      //  std::cout << "lm size: " << lm->getSize() << std::endl;
       // std::cout << "Pw_left_edge: " << Pw_left_edge << std::endl;
       // std::cout << "Pw_right_edge: " << Pw_right_edge << std::endl;

        cv::Mat uv_left;
        ProjectLandMark(Pw_left_edge,  uv_left);
        cv::Mat uv_right;
        ProjectLandMark(Pw_right_edge,  uv_right);

      //  std::cout << "uv_left: " << uv_left << std::endl;
      //  std::cout << "uv_right: " << uv_right << std::endl;

        float size_pixels  = uv_right.at<float>(0,0)- uv_left.at<float>(0,0);
       // std::cout << "size_pixels: " << size_pixels << std::endl;
        return size_pixels;

    }
}

void Frame::propagateTracking(Frame &frame_previous){
    matches.propagateTracking(frame_previous.getLandMarkMatches());
}

//clear all mappoint to keypoint associations
int Frame::clearAssociations(){
 return matches.clearAssociations();
}

bool Frame::isOutlier(int i) const {
    return matches.isOutlier(i);
}
int  Frame::setOutlier(int i, bool is_outlier){
    return matches.setOutlier(i, is_outlier);
}

std::vector<MapPoint*>  Frame::replicatemvpMapPoints() const
{
    std::vector<MapPoint*> mvp_sim;
    for(int i = 0; i < N; ++i){
        mvp_sim.push_back(matches.hasAssociation(i));
    }
 //   validateNewAssociations(mvp_sim);
    return mvp_sim;
}

void Frame::CalcRelativeQuat()
{
    if(sensor_data.isImuValid()){
	   std::vector<double> vqref = KeyFrame::GetRefQuat();
	   Eigen::Quaterniond q = Converter::toQuatEigen(sensor_data.getQuat() );
	   Eigen::Quaterniond qref  = Converter::toQuatEigen(vqref);
	   Eigen::Quaterniond quatRel = qref.inverse()*q;
	   ImuData imud = sensor_data.getImu();
       imud.quat_rel = Converter::toQuatStdvec(quatRel);
       imud.quat_origin = vqref;
       sensor_data.setImu(imud);
   }
   else {
    //   std::cout << "no valid imu data, cannot Frame::CalcRelativeQuat()" << std::endl;
   }
}

std::vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = std::min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = std::min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = views.keypt(vCell[j]);
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

std::vector<size_t> Frame::GetFeaturesInAreaNEW(const float &x, const float  &y, const float  &r) const{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = std::min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = std::min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = views.keypt(vCell[j]);
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
     //   std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(views.getDescriptors());
        std::vector<FeatureDescriptor> descriptors =  views.getDescriptors();
        feature_vocabulary->transform(descriptors, mBowVec, mFeatVec, 4);
    }
}

cv::Mat Frame::UnprojectStereo(const int &i) //move into camera class
{
    const float z = views.depth(i);
    if(z>0)
    {
        const float u = views.keypt(i).pt.x;
        const float v = views.keypt(i).pt.y;
        cv::Mat x3Dc = camera.Unproject(u,v, z);

        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

/*
int Frame::predictScale(const float &currentDist, MapPoint* pMP){

    float ratio = pMP->GetMaxDistanceInvariance()/(1.2*currentDist);  //(1.2 factor is oddity from GetMaxDistanceInvariance())
    int nScale_this = ceil(log(ratio)/views.orbParams().mfLogScaleFactor);
    int nScales = views.orbParams().mnScaleLevels;
    if(nScale_this <0)
        nScale_this = 0;
    else if(nScale_this >=nScales)
        nScale_this = nScales-1;

    return nScale_this;
}
*/
void Frame::validateMatches(){
    std::map<MapPoint*, int> counts;
    for(auto it = matches.begin(); it != matches.end(); ++it){
        MapPoint* pMP = it->second;
        if(counts.count(pMP) == 1){
            counts[pMP] = counts[pMP]+1;
        } else {
            counts[pMP] = 1;
        }
    }

    std::cout << "validating frame: " << mnId << std::endl;
    for(auto mit = counts.begin(); mit != counts.end(); ++mit){
        if(mit->second > 1){
            std::cout << "pMP: " << mit->first->mnId << " has associations : " << mit->second << std::endl;
        }
    }
}

int Frame::validateNewAssociations(std::vector<MapPoint*> mvpMapPoints) const {
    int errors_nm = 0;
    int errors_na = 0;
    int errors_a = 0;
    int correct = 0;
    for(int i  = 0; i < N; ++i){
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP){
            /*
            int cor_idx= hasAssociation(pMP);
            if( cor_idx < 0 ){
              ++errors_na;
            }
            if(cor_idx != i){
        //      std::cout << "new association validation, expected idx: " << i << ", got: " << cor_idx << std::endl;
              ++errors_nm;
            }
            */
            MapPoint* mpt_cor = hasAssociation(i);
            if(!mpt_cor){
                ++errors_na;
            }
            else if(mpt_cor->mnId != pMP->mnId)
            {
                ++errors_nm;
                //    std::cout << "expected mpt id: " << pMP->mnId << ", instead got: " << mpt_cor->mnId << std::endl;
            }
            else{
                //     std::cout << "matched: " << pMP->mnId << ", to: " << mpt_cor->mnId << std::endl;
                ++correct;
            }
        } else {
            if(hasAssociation(i)){
                ++errors_a;
            }
        }

    }
    std::cout << "validating mvp simulated for Frame id: " << mnId << " errors, indexs don't match: " << errors_nm <<" , no assocation in new struct: "<< errors_na << "  , empty mappoint error: "  << errors_a << ", correct: " << correct<< std::endl;
    return errors_nm + errors_na + errors_a;
}


} //namespace ORB_SLAM
