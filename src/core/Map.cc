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

//Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
//{
//}

Map::Map(std::shared_ptr<KeyFrameDB> keyframe_db_, std::shared_ptr<MapPointDB> mappoint_db_ ):
 keyframe_db_local(keyframe_db_), mappoint_db_local(mappoint_db_),mnMaxKFid(0),mnBigChangeIdx(0)
{
//    keyframe_db_local = std::make_shared<KeyFrameDB>();
//    keyframe_db_local->setVocab(keyframe_db_parent->getVocab());
//
//    mappoint_db_local = std::make_shared<MapPointDB>();

}

Map::Map(std::shared_ptr<Map> parent) {
    parent_map = parent;
    keyframe_db_parent = parent->keyframe_db_local;
    mappoint_db_parent = parent->mappoint_db_local;

    keyframe_db_local = std::make_shared<KeyFrameDB>();
    keyframe_db_local->setVocab(keyframe_db_parent->getVocab());

    mappoint_db_local = std::make_shared<MapPointDB>();
    mnMaxKFid = parent->mnMaxKFid;
}

std::shared_ptr<Map> Map::createSubMap(bool set_active) {
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::shared_ptr<Map> newmap =  std::make_shared<Map>(shared_from_this());
    sub_maps.push_back(newmap); //do this first after creating map!

    if(set_active) {
        newmap->setActive();
    }
    return newmap;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
//    std::cout << "to map, adding KF: " << pKF->mnId << " from cam: " << pKF->camera.camName << std::endl;
    //Map* map_root = getRoot();
   // pKF->setMap(map_root);
    //map_root->_addKeyFrame_(pKF);
    _addKeyFrame_(pKF);
}

bool Map::_addKeyFrame_(KeyFrame *pKF) {
    if(isActive()) {
        std::unique_lock<std::mutex> lock(mMutexMap);
        if(!firstKFadded){
           // firstKFid = pKF->mnId;
           KFs_nonerasable.insert(pKF);
            firstKFadded = true;
        }
        keyframe_db_local->add(pKF);
        if(pKF->mnId>mnMaxKFid) {
            mnMaxKFid = pKF->mnId;
        }
        return true;
    }
    else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it) {
            if((*it)->_addKeyFrame_(pKF)){
                return true;
            }
        }
    }
    return false;
}


bool Map::EraseKeyFrame(KeyFrame *pKF)
{
 //   Map* map_root = getRoot();
 //   return map_root->_eraseKeyFrame_(pKF);
    return _eraseKeyFrame_(pKF);
}

bool Map::_eraseKeyFrame_(KeyFrame *pKF) {

    if(keyframe_db_local->exists(pKF)) {
        std::unique_lock<std::mutex> lock(mMutexMap);
        keyframe_db_local->erase(pKF, "All");
        return true;
    }
    else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_eraseKeyFrame_(pKF)){
                return true;
            }
        }
    }
    return false;
}

bool Map::isKFErasable(KeyFrame *pKF) {
    bool erasable = true;
    _isKFErasable_(pKF, erasable);
    return erasable;
}

bool Map::_isKFErasable_(KeyFrame *pKF, bool &erasable) {
    //don't allow first keyframe added to each map to be erased. it's the reference point relative to other maps and origin of world frame for first map
    //return value is whether pKF has been found in a map and so searching can terminate.
    // need to work through all submaps to check (can't just find the submap in which pKF i sin the keyframe_db b/c that keyframe_db may be connected to other submap keyframedbs  might be
    // best to push "isErasable" down to keyframe db )

    if(KFs_nonerasable.count(pKF)){
        erasable = false;
        return true;  //indicates there's no need to search further
    } else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_isKFErasable_(pKF, erasable)){
                return true;
            }
        }
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

void Map::SetBadKeyFrame(KeyFrame* pKF) {
    if(!isKFErasable(pKF)){ //don't delete the first keyframe in each submap
        return;
    } else if (pKF->isProtected()) {  //LoopClosing can protect a keyframe by setting mbNotErase, if set don't erase, but mark it to be erased later by setting mbToBeErased
        pKF->mbToBeErased = true;
        return;
    }

    //   std::cout << "erasing landmark associations for KF: " << pKF->mnId << ", named: " << pKF->kfImgName << std::endl;
    // do i need to set it bad immediately so that no other associations will be made !!!!
    std::set<MapPoint *> spMP = pKF->GetMapPoints();
    for (auto it = spMP.begin(); it != spMP.end(); ++it) {
        //mappoint_db.eraseObservation(*it, pKF);
        MapPoint *pMP = *it;
        eraseAssociation(pKF, pMP);
    }
    EraseKeyFrame(pKF);

}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
  //  if(registered) {
  //      return keyframe_db->getNumberOfKeyFrames();
  //  }
  //  else {
 //       return keyframe_db_local->getNumberOfKeyFrames();
 //   }
    return keyframe_db_local->getNumberOfKeyFrames();
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
//    std::set<KeyFrame*> all_kfs;
//    if(registered){
//        all_kfs = keyframe_db_global->getAllKeyFrames();
//    } else {
//        all_kfs = keyframe_db_local->getAllKeyFrames();
//    }
    std::set<KeyFrame*> all_kfs =keyframe_db_local->getAllKeyFrames();
    return std::vector<KeyFrame*>(all_kfs.begin(), all_kfs.end());
}

std::vector<KeyFrame *> Map::getAllKeyFramesIncludeSubmaps() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::set<KeyFrame*> all_kfs = _getAllKeyFramesIncludeSubmaps_();

    return std::vector<KeyFrame*>(all_kfs.begin(), all_kfs.end());
}

std::set<KeyFrame *> Map::_getAllKeyFramesIncludeSubmaps_(){
    std::set<KeyFrame*> all_kfs =keyframe_db_local->getAllKeyFrames();

    for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
        std::set<KeyFrame*> submap_kfs = (*it)->_getAllKeyFramesIncludeSubmaps_();
        all_kfs.insert(submap_kfs.begin(), submap_kfs.end());
    }
    return all_kfs;
}
void Map::AddMapPoint(MapPoint *pMP,KeyFrame* pKF_ref, int idx)
{
    pKF_ref->associateLandMark(idx, pMP, true);
//    Map* map_root = getRoot();
//    map_root->_addMapPoint_(pMP, pKF_ref, idx);
    _addMapPoint_(pMP, pKF_ref, idx);
}

bool Map::_addMapPoint_(MapPoint *pMP, KeyFrame *pKF_ref, int idx) {
    if(isActive()) {
        std::unique_lock<std::mutex> lock(mMutexMap);
        mappoint_db_local->addEntry(pMP, pKF_ref, idx);
        return true;
    }
    else{
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it) {
            if((*it)->_addMapPoint_(pMP, pKF_ref, idx)){
                return true;
            }
        }
    }
    return false;
}

MapPoint*  Map::newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx){
    MapPoint* pMP = MPfactory.construct(Pos, pKF, idx);
    AddMapPoint(pMP, pKF, idx);
    return pMP;
}


bool Map::eraseMapPoint(MapPoint *pMP)
{
//    Map* map_root = getRoot();
//    return map_root->_eraseMapPoint_(pMP);

    return _eraseMapPoint_(pMP);
    // TODO: This only erases records of  the pointer.
    // Delete the MapPoint
}

bool Map::_eraseMapPoint_(MapPoint *pMP) {
    if(mappoint_db_local->exists(pMP)) {
        std::unique_lock<std::mutex> lock(mMutexMap);
        mappoint_db_local->eraseEntry(pMP);
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_eraseMapPoint_(pMP)){
                return true;
            }
        }
    }

    return false;
}

int Map::replaceMapPoint(MapPoint* pMP_old, MapPoint* pMP_new){
    int res = mappoint_db_local->replace(pMP_old, pMP_new);
    if(res <0){ //did not succeed
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            int res2 = (*it)->replaceMapPoint(pMP_old, pMP_new);
            if(res2 >=0){
                return res2;
            }
        }
    } else {
        return res;
    }
    return -1; // failed
}


void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mappoint_db_local->numMapPoints();
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

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::set<MapPoint*> all_mpts = mappoint_db_local->getAllMapPoints();
    return std::vector<MapPoint*>(all_mpts.begin(), all_mpts.end());

}

std::vector<MapPoint *> Map::getAllMapPointsIncludeSubmaps() {
    std::set<MapPoint *> all_mpts = _getAllMapPointsIncludeSubmaps_();
    return std::vector<MapPoint *>(all_mpts.begin(), all_mpts.end());
}

std::set<MapPoint *> Map::_getAllMapPointsIncludeSubmaps_() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::set<MapPoint *> all_mpts = mappoint_db_local->getAllMapPoints();

    for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
        std::set<MapPoint *> submap_mpts = (*it)->_getAllMapPointsIncludeSubmaps_();
        all_mpts.insert(submap_mpts.begin(), submap_mpts.end());
    }
    return all_mpts;
}

std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}


int Map::addAssociation(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace){
    if(pKF->isBad()){
        return -1;
    }

    //two potential replacement issues: 1: idx is already associated with another mappoint; 2: mappoint is already associated with another idx. #2 is handled at lower level by propagating "replace"
    // in case 1 need to manually remove old association in mappoint_db
    MapPoint* pMP_old;
    pKF->associateLandMark(idx, pMP, replace, pMP_old); //should run this through keyframedb so that it can track changing assocations and update covis etc as needed

    if(pMP_old && replace){ //remove old association first in case pMP_old = pMP;
       eraseAssociation(pKF, pMP_old);
        //mappoint_db_local->eraseObservation(pMP, pKF);  //previous line is much better! just doing this to test consistency with previous implementation
    }

  //  Map* map_root = getRoot();
  //  bool res = map_root->_addAssociation_(pKF, idx, pMP, replace);
    bool res = _addAssociation_(pKF, idx, pMP, replace);
    if(res){
        return 0;
    } else {
        return -1;
    }
}


bool Map::_addAssociation_(KeyFrame *pKF, int idx, MapPoint *pMP, bool replace) {
    if(mappoint_db_local->exists(pMP)) {
        mappoint_db_local->addObservation(pMP, pKF, idx, replace);
        mappoint_db_local->updateEntry(pMP);
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_addAssociation_(pKF, idx, pMP, replace)){
                return true;
            }
        }
    }

    return false;
}



int Map::eraseAssociation(KeyFrame* pKF,  MapPoint* pMP){
    pKF->removeLandMarkAssociation(pMP);

//    Map* map_root = getRoot();
//    bool res = map_root->_eraseAssociation_(pKF, pMP);
    bool res = _eraseAssociation_(pKF, pMP);

    if(res){
        return 0;
    } else {
        return -1;
    }

}

bool Map::_eraseAssociation_(KeyFrame *pKF, MapPoint *pMP) {
   // std::cout << "ABOUT TO test exists in _eraseAssociation_ from: " << this << std::endl;
    if(mappoint_db_local->exists(pMP)){
        int res = mappoint_db_local->eraseObservation(pMP, pKF);
        if(res ==0){
            mappoint_db_local->updateEntry(pMP);
        }
        return true;
    }

    else{
   //     std::cout << "ABOUT TO CHECK SUBMAPS in _eraseAssociation_ from: " << this << std::endl;
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_eraseAssociation_(pKF, pMP) ){
                return true;
            }
        }
    }

    return false;
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


void Map::clear()
{
    //pass call to submaps
    for(auto it = sub_maps.begin(); it!= sub_maps.end(); ++it){
        (*it)->clear();
    }

    std::set<MapPoint*> mpts_all = mappoint_db_local->getAllMapPoints();
    for(std::set<MapPoint*>::iterator sit=mpts_all.begin(), send=mpts_all.end(); sit!=send; sit++) {
        delete *sit;
    }

    std::set<KeyFrame*> keyframes_local2 = keyframe_db_local->getAllKeyFrames();
    for(std::set<KeyFrame*>::iterator sit=keyframes_local2.begin(), send=keyframes_local2.end(); sit != send; sit++) {
         delete *sit;
    }

   // for(std::set<KeyFrame*>::iterator sit=keyframes_local.begin(), send=keyframes_local.end(); sit != send; sit++) {
   //     delete *sit;
   // }

//    pkeyframe_db->clear();
    keyframe_db_local->clear();
    mappoint_db_local->clear();
   // keyframes_local.clear();
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}



bool Map::isActive() const {
    return active;
}

void Map::setActive() {
    //NEED TO MAKE SURE NO OTHER MAP IN TREE IS ACTIVE
    setActive(shared_from_this());
}

void Map::setActive(std::shared_ptr<Map> active_map) {
    std::shared_ptr<Map> root = getRoot();
//    std::cout << "in setActive(active_map), start at: " << root << ", looking to set active: " << active_map << std::endl;
    root->_setActive_(active_map);

}

void Map::_setActive_(std::shared_ptr<Map> active_map) {
    //walk through map tree setting all maps except "active_map" as not active.
    //alternate idea is to create an "ActiveToken" class, create a unique ptr to one for each tree and pass it around via move (but get's tricky when trees are merged)
    if(shared_from_this() == active_map){
        active = true;
        std::cout << "setting active map to: " << shared_from_this() << std::endl;
    } else{
        active = false;
    }

    for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
        (*it)->_setActive_(active_map);
    }
}

std::shared_ptr<Map> Map::getRoot() {
    std::shared_ptr<Map> map_root = shared_from_this();
    while(map_root->getParentMap()){  //find root of map tree
        map_root = map_root->getParentMap();
    }
    return map_root;
}

void Map::registerWithParent() {
    if(parent_map) { //root will not have a parent and can't be registered with anything
        mappoint_db_parent->addChild(mappoint_db_local);
        keyframe_db_parent->addChild(keyframe_db_local);
        registered = true;
    }
}

std::shared_ptr<Map> Map::getParentMap() const {
    return parent_map;
}

void Map::setParentMap(std::shared_ptr<Map> parentMap) {
    parent_map = parentMap;
}

std::vector<KeyFrame *> Map::getVectorCovisibleKeyFrames(KeyFrame *pKF) {
    if(keyframe_db_local->exists(pKF)){
        return keyframe_db_local->GetVectorCovisibleKeyFrames(pKF);
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            std::vector<KeyFrame *> covisKFs = (*it)->getVectorCovisibleKeyFrames(pKF);
            if(!covisKFs.empty()){
                return covisKFs;
            }
        }
    }
    return std::vector<KeyFrame *>();
}

std::vector<KeyFrame *> Map::getBestCovisibilityKeyFrames(KeyFrame *pKF, const int &N) {
    if(keyframe_db_local->exists(pKF)){
        return keyframe_db_local->GetBestCovisibilityKeyFrames(pKF, N);
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            std::vector<KeyFrame *> covisKFs = (*it)->getBestCovisibilityKeyFrames(pKF, N);
            if(!covisKFs.empty()){
                return covisKFs;
            }
        }
    }
    return std::vector<KeyFrame *>();
}

bool Map::update(KeyFrame *pKF) {
    if(keyframe_db_local->exists(pKF)){
        return keyframe_db_local->update(pKF);
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->update(pKF)){
                return true;
            }
        }
    }
    return false;
}

std::set<KeyFrame *> Map::getSpanningTreeChildren(KeyFrame *pKF) {
    std::set<KeyFrame *> children;
    _getSpanningTreeChildren_(pKF, children);
    return children;
}

bool Map::_getSpanningTreeChildren_(KeyFrame *pKF, std::set<KeyFrame *> &children) {
    if(keyframe_db_local->exists(pKF)){
        children =  keyframe_db_local->getSpanningTreeChildren(pKF);
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_getSpanningTreeChildren_(pKF, children)){
                return true;
            }
        }
    }
    return false;
}

std::set<KeyFrame *> Map::detectRelocalizationCandidates(Frame *F) {
    std::set<KeyFrame *> candidates = keyframe_db_local->DetectRelocalizationCandidates(F);
    for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it) {
        std::set<KeyFrame *> addn_candidates = (*it)->detectRelocalizationCandidates(F);
        candidates.insert( addn_candidates.begin(), addn_candidates.end());
    }
    return candidates;
}

bool Map::update(MapPoint *pMP) {
    if(mappoint_db_local->exists(pMP)) {
        mappoint_db_local->updateEntry(pMP);
        return true;
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->update(pMP)){
                return true;
            }
        }
    }
    return false;
}

bool Map::setNonErasable(KeyFrame *pKF) {
    if(keyframe_db_local->exists(pKF)){
        std::unique_lock<std::mutex> lock(mMutexMap);
        if(!pKF->isBad()){
            KFs_nonerasable.insert(pKF);
            return true;
        } else {
            return false;
        }
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->setNonErasable(pKF)){
                return true;
            }
        }
    }
    return false;
}

const Tse3Parent &Map::getTse3Parent() const {
    return tse3parent;
}

void Map::setTse3Parent(const Tse3Parent &tse3Parent) {
    tse3parent = tse3Parent;
}

bool Map::isLocalOrigin(KeyFrame *pKForg, Tse3Parent &tse3Parent_) {
    bool is_origin = false;
    _isLocalOrigin_(pKForg, is_origin, tse3Parent_);
    return is_origin;
}

bool Map::_isLocalOrigin_(KeyFrame *pKForg, bool &is_origin, Tse3Parent &tse3Parent_) {
    if(keyframe_db_local->isOwned(pKForg)) {
        if(pKForg == tse3parent.pKFref_this) {
            tse3Parent_ = tse3parent;
            is_origin = true;
        } else {
            is_origin = false;
        }
        return true;  //found relevant submap
    } else {
        for(auto it = sub_maps.begin(); it != sub_maps.end(); ++it){
            if((*it)->_isLocalOrigin_(pKForg, is_origin, tse3Parent_)){
                return true;
            }
        }
    }
    return false;
}




} //namespace ORB_SLAM
