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

Map::Map(Map* parent, std::shared_ptr<KeyFrameDB> keyframe_db_, std::shared_ptr<MapPointDB> mappoint_db_ ):
parent_map(parent), keyframe_db(keyframe_db_), mappoint_db(mappoint_db_), mnMaxKFid(0), mnBigChangeIdx(0)
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
    if(active_map == this) {
        pKF->setMap(this);
        keyframe_db_local.add(pKF);
        if(registered){
            keyframe_db->add(pKF);
        }

       // keyframes_local.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }
    else{
        active_map->AddKeyFrame(pKF);
    }
}


bool Map::EraseKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    if(keyframe_db_local.exists(pKF)) {
        keyframe_db_local.erase(pKF, "All");
        if(registered){
            keyframe_db->erase(pKF, "All");
        }
        return true;
    }
    else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->EraseKeyFrame(pKF)){
                return true;
            }
        }
        //pass up to parent? if not found in children?????
    }
    return false;
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

    std::cout << "erasing landmark associations for KF: " << pKF->mnId << ", named: " << pKF->kfImgName << std::endl;
    // do i need to set it bad immediately so that no other associations will be made !!!!
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
   keyframe_db->setVocab(pVoc);
}
std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::set<KeyFrame*> all_kfs = keyframe_db->getAllKeyFrames();
    return std::vector<KeyFrame*>(all_kfs.begin(), all_kfs.end());
  // return std::vector<KeyFrame*>(keyframes_local.begin(), keyframes_local.end());
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
    if(active_map == this){
        mappoint_db_local.addEntry(pMP, pKF_ref, idx);
        if(registered) {
            mappoint_db->addEntry(pMP, pKF_ref, idx);
        }
    } else{
        active_map->AddMapPoint(pMP,pKF_ref, idx);
    }

}

MapPoint*  Map::newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx){
   MapPoint* pMP = MPfactory.construct(Pos, pKF, idx);
   AddMapPoint(pMP, pKF, idx);
   return pMP; 
}


bool Map::eraseMapPoint(MapPoint *pMP)
{
    if(mappoint_db_local.exists(pMP)) {
        mappoint_db_local.eraseEntry(pMP);
        if(registered) {
            mappoint_db->eraseEntry(pMP);
        }
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->eraseMapPoint(pMP)){
                return true;
            }
        }
    }

    return false;

    // TODO: This only erases records of  the pointer.
    // Delete the MapPoint
}

int Map::replaceMapPoint(MapPoint* pMP_old, MapPoint* pMP_new){
     return mappoint_db->replace(pMP_old, pMP_new);
/// STILL WORKING ON THIS !!!!!
    if(mappoint_db_local.exists(pMP)) {
        mappoint_db_local.eraseEntry(pMP);
        if(registered) {
            mappoint_db->eraseEntry(pMP);
        }
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->eraseMapPoint(pMP)){
                return true;
            }
        }
    }

    return false;


}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    return mappoint_db->getAllMapPoints();
}

long unsigned int Map::MapPointsInMap()
{
    return mappoint_db->numMapPoints();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return  keyframe_db->getNumberOfKeyFrames();
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
    if(pKF->isBad()){
        return -1;
    }

    //two potential replacement issues: 1: idx is already associated with another mappoint; 2: mappoint is already associated with another idx. #2 is handled at lower level by propagating "replace"
    // in case 1 need to manually remove old association in mappoint_db
     MapPoint* pMP_old;
     pKF->associateLandMark(idx, pMP, replace, pMP_old); //should run this through keyframedb so that it can track changing assocations and update covis etc as needed
    //note: the above line will get called multiple times if submaps are searched. correct operation of this function (addAsssocaition) relies on the fact that pMP_old will only get assigned a value
    // if pMP_old != pMP; this is not ideal but is done to avoid having to search upward in map graph structure (i.e. going up to parents).

    if(pMP_old && replace){ //remove old association first in case pMP_old = pMP;
        //std::cout << "removing old observation, which would have lingered in past" << std::endl;
        eraseAssociation(pKF, pMP_old);
    }

    if(mappoint_db_local.exists(pMP)) {
        mappoint_db_local.addObservation(pMP, pKF, idx, replace);
        mappoint_db_local.updateEntry(pMP);
        if(registered) {
            mappoint_db->addObservation(pMP, pKF, idx, replace);
            mappoint_db->updateEntry(pMP);
        }
        return 0;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->addAssociation(pKF, idx, pMP, replace) == 0){
                return 0;
            }
        }
    }

    return -1;
}

int Map::eraseAssociation(KeyFrame* pKF,  MapPoint* pMP){


    if(mappoint_db_local.exists(pMP)){
        pKF->removeLandMarkAssociation(pMP);
        bool erased = mappoint_db_local.eraseObservation(pMP, pKF);

        if(erased){
        } else {
            mappoint_db_local.updateEntry(pMP);
        }

        if(registered){
            bool erased_global = mappoint_db->eraseObservation(pMP, pKF);

            if(erased_global){
            } else {
                mappoint_db->updateEntry(pMP);
            }
        }

        return 0;
    }

    else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->eraseAssociation(pKF, pMP) == 0){
                return 0;
            }
        }
    }

    return -1;
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
    //pass call to submaps
    for(auto it = sub_maps.begin(); it!= sub_maps.end(); ++it){
        (*it)->clear();
    }

    std::vector<MapPoint*> mpts_all = GetAllMapPoints();
    for(std::vector<MapPoint*>::iterator sit=mpts_all.begin(), send=mpts_all.end(); sit!=send; sit++) {
        delete *sit;
    }

    std::set<KeyFrame*> keyframes_local2 = keyframe_db_local.getAllKeyFrames();
    for(std::set<KeyFrame*>::iterator sit=keyframes_local2.begin(), send=keyframes_local2.end(); sit != send; sit++) {
         delete *sit;
    }

   // for(std::set<KeyFrame*>::iterator sit=keyframes_local.begin(), send=keyframes_local.end(); sit != send; sit++) {
   //     delete *sit;
   // }

//    pkeyframe_db->clear();
    keyframe_db_local.clear();
    mappoint_db_local.clear();
   // keyframes_local.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

    void Map::createSubMap() {
        sub_maps.push_back(std::make_unique<Map>(this, keyframe_db, mappoint_db));
    }
/*
void Map::validateCovisiblityGraph(){
    pkeyframe_db->validateCovisiblityGraph();
}
*/
} //namespace ORB_SLAM
