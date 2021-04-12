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

#include "Map.h"
#include <KeyFrameDB.h>
#include <MapPointDB.h>

namespace HYSLAM
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

/*
void Map::setKeyFrameDB(KeyFrameDB* pKFDB){
    pkeyframe_db = pKFDB;
}
*/
void Map::AddKeyFrame(KeyFrame *pKF)
{
//    std::cout << "to map, adding KF: " << pKF->mnId << " from cam: " << pKF->camera.camName << std::endl;
    std::unique_lock<std::mutex> lock(mMutexMap);
   // pkeyframe_db->add(pKF);
   pKF->setMap(this);
   keyframe_db.add(pKF);

    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}


void Map::EraseKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
  //  pkeyframe_db->erase(pKF, "All");  
    keyframe_db.erase(pKF, "All");
    mspKeyFrames.erase(pKF);
}

void Map::ClearKeyFrameProtection(KeyFrame* pKF){

    if(pKF->GetLoopEdges().empty())
    {
        pKF->clearProtection();
    }


    if(pKF->mbToBeErased)
    {
        SetBadKeyFrame(pKF);
    }

}

void Map::SetBadKeyFrame(KeyFrame* pKF){

    if(pKF->mnId == 0  ){
        return ;
    }

    else if(pKF->isProtected()){  //LoopClosing can protect a keyframe by setting mbNotErase, if set don't erase, but mark it to be erased later by setting mbToBeErased
        pKF->mbToBeErased = true;
        return;
    }

    std::cout << "erasing mapts for KF: " << pKF->mnId << std::endl;
    std::set<MapPoint*> spMP = pKF->GetMapPoints();
    for(auto it = spMP.begin(); it != spMP.end(); ++it) {
        //mappoint_db.eraseObservation(*it, pKF);
        MapPoint* pMP = *it;
        eraseAssociation(pKF, pMP);
    }
    EraseKeyFrame(pKF);
}


void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

void Map::setKeyFrameDBVocab(FeatureVocabulary* pVoc){
  //  pkeyframe_db->setVocab(pVoc);
   keyframe_db.setVocab(pVoc);
}
std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}
/*
void Map::setMapPointDB(MapPointDB* pMPDB){
    pmappoint_db = pMPDB;
}
*/
void Map::AddMapPoint(MapPoint *pMP,KeyFrame* pKF_ref, int idx)
{
    pKF_ref->associateLandMark(idx, pMP, true);
    std::unique_lock<std::mutex> lock(mMutexMap);
    mappoint_db.addEntry(pMP, pKF_ref, idx);
}

MapPoint*  Map::newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx){
   MapPoint* pMP = MPfactory.construct(Pos, pKF, idx);
   AddMapPoint(pMP, pKF, idx);
   return pMP; 
}


void Map::eraseMapPoint(MapPoint *pMP)
{
    mappoint_db.eraseEntry(pMP);
    // TODO: This only erases records of  the pointer.
    // Delete the MapPoint
}

int Map::replaceMapPoint(MapPoint* pMP_old, MapPoint* pMP_new){
     return mappoint_db.replace(pMP_old, pMP_new);
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    return mappoint_db.getAllMapPoints();
}

long unsigned int Map::MapPointsInMap()
{
    return mappoint_db.numMapPoints();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

void  Map::visibleMapPoints(KeyFrame* pKFi, std::vector<MapPoint*> &visible_mpts){
  //finds all mappoints visible in the given keyframe
  //TO DO: acclerate with BVH or similar
  std::vector<MapPoint*> mpts_all = GetAllMapPoints();
  for(std::vector<MapPoint*>::iterator mit =  mpts_all.begin(); mit !=  mpts_all.end(); ++mit){
    MapPoint* mpt = *mit;
 //   cv::Mat mpt_pos = mpt->GetWorldPos();
    if(pKFi->isLandMarkVisible(mpt)){
      visible_mpts.push_back(mpt);
    }
  }
   
}

int Map::addAssociation(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace){
     pKF->associateLandMark(idx, pMP, true); //should run this through keyframedb so that it can track changing assocations and update covis etc as needed
     mappoint_db.addObservation(pMP,pKF, idx );
     mappoint_db.updateEntry(pMP);
     return 0;
}

int Map::eraseAssociation(KeyFrame* pKF,  MapPoint* pMP){
    pKF->removeLandMarkAssociation(pMP);
  //  bool erased = pmappoint_db->eraseObservation(pMP, pKF);
    bool erased = mappoint_db.eraseObservation(pMP, pKF);    
    if(erased){
   //     EraseMapPoint(pMP);
    } else {
      //  pmappoint_db->updateEntry(pMP); 
        mappoint_db.updateEntry(pMP); 
    }
    return 0;
}
/*
int  Map::eraseAssociation(KeyFrame* pKF,  int idx){
  return -1;
}
*/
std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    std::vector<MapPoint*> mpts_all = GetAllMapPoints();
    for(std::vector<MapPoint*>::iterator sit=mpts_all.begin(), send=mpts_all.end(); sit!=send; sit++) {
        delete *sit;
    }

    for(std::set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++) {
        delete *sit;
    }

//    pkeyframe_db->clear();
    keyframe_db.clear();
    mappoint_db.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}
/*
void Map::validateCovisiblityGraph(){
    pkeyframe_db->validateCovisiblityGraph();
}
*/
} //namespace ORB_SLAM
