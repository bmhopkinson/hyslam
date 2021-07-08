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

#ifndef MAP_H
#define MAP_H

/*
 * the map is composed of KeyFrame and LandMarks (mappoints). LandMarks are viewed in KeyFrames.
 * information about KeyFrames is stored in a KeyFrameDB
 * information about LandMarks is store in a MapPointDB
 * most functions calls are simply propagated to KeyFrameDB or MapPointDB
 *
 * KeyFrames are never really deleted from memory they're just removed from the KeyFrameDB - this should be fixed
 *  have an idea of making this a recursive data structure so we can have submaps of finite size related to each other by rigid transforms
 *   would enable easier reinitialization when tracking is lost (simply create new submap), breaking up enormously large maps that take forever for global bundle adjustment, etc
 *
 *   key functionality:
 *   AddKeyFrame(KeyFrame* pKF) - adds KeyFrame to KeyFrameDB and sets this map as the KeyFrame's Map
 *   EraseKeyFrame(KeyFrame* pKF) - erases KeyFrame from KeyFrameDB, doesn't delete it
 *   SetBadKeyFrame(KeyFrame* pKF) - first removes all associations between pKF and viewed LandMarks then calls EraseKeyFrame(pKF)
 *   getKeyFrameDB() - returns underlying KeyFrameDB - should eliminate this so implementation isn't exposed - it isn't used much
 *   getMapPointDB()- returns underlying MapPointDB - should eliminate this so implementation isn't exposed - it isn't used much
 *   AddMapPoint(MapPoint* pMP, KeyFrame* pKF_ref, int idx) - mappoint must be associated with at least one keyframe (refrence keyframe)
 *          so here new Mappoint is associated with keyframe and added to MappointDB
 *    MapPoint* newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx) - construct new mappoint (and return pointer to it), then call AddMapPoint(MapPoint* pMP, KeyFrame* pKF_ref, int idx)
 *    visibleMapPoints(KeyFrame* pKFi, std::vector<MapPoint*> &visible_mpts) - all mappoints visible to pKFi - done by brute force projection all of mappoint in map into pKFi
 *          really should accelrate with a Bounding Volume Hierarchy or something similar
 *    int addAssociation(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace); - associates Feature idx in pKF with LandMark pMP.
 *          this is done directly on pKF - really should add functionality so that it can go through KeyFrameDB
 *          with LandMark is done (appropriately) through MapPointDB
 *    int eraseAssociation(KeyFrame* pKF,  MapPoint* pMP) removes association between pKF and LandMark pMP.
 *          this is done directly on pKF - really should add functionality so that it can go through KeyFrameDB
 *          with LandMark is done (appropriately) through MapPointDB
 *
 */

#include <MapPoint.h>
#include <KeyFrame.h>
#include <FeatureVocabulary.h>
#include <KeyFrameDB.h>
#include <MapPointDB.h>
#include <MapPointFactory.h>

#include <opencv2/core/core.hpp>

#include <set>
#include <mutex>
#include <memory>

namespace HYSLAM
{


class Map
{
public:
  //  Map();

    Map(std::shared_ptr<KeyFrameDB> keyframe_db_, std::shared_ptr<MapPointDB> mappoint_db_); //constuctor for NEW map
    Map(Map* parent); //constructor for submap

    //multimap functions
    void createSubMap();
    Map *getParentMap() const;
    void setParentMap(Map *parentMap);
    void registerWithParent();
    Map* getRoot();
    bool isActive() const;
    void setActive(bool active);

    // KeyFrame functions
    void AddKeyFrame(KeyFrame* pKF);
    bool EraseKeyFrame(KeyFrame* pKF);  //details on erasing KFs is complicated due to multithreading/loop closing - probably should rename these functions
    void ClearKeyFrameProtection(KeyFrame* pKF);   //used once loop closing is done w/ a keyframe to allow its removal, and to erase it if it was marked for removal during the loop closing attempt - this isn't the best - would be better to clearprotection directly on keyframe, add keyframes to be deleted to a queue for later deletion once protection is cleared
    void SetBadKeyFrame(KeyFrame* pKF);
 //   void validateCovisiblityGraph();
    KeyFrameDB* getKeyFrameDB(){return keyframe_db_local.get();}
    long unsigned  KeyFramesInMap();
    std::vector<KeyFrame*> GetAllKeyFrames();
    
    //mappointDB functions
    MapPointDB* getMapPointDB(){ return mappoint_db_local.get(); }
    void AddMapPoint(MapPoint* pMP, KeyFrame* pKF_ref, int idx);
    MapPoint* newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx); 
    bool eraseMapPoint(MapPoint* pMP);
    int replaceMapPoint(MapPoint* pMP_old, MapPoint* pMP_new);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    long unsigned int MapPointsInMap();
    void visibleMapPoints(KeyFrame* pKFi, std::vector<MapPoint*> &visible_mpts);
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    //Associations
    int addAssociation(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace);
    int eraseAssociation(KeyFrame* pKF,  MapPoint* pMP);
 //   int eraseAssociation(KeyFrame* pKF,  int idx);  //NOT YET IMPLEMENTED


    void InformNewBigChange();
    int GetLastBigChangeIdx();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;


protected:

  //  KeyFrameDB keyframe_db;
  //  MapPointDB mappoint_db;
  //  std::shared_ptr<KeyFrameDB> keyframe_db_global = nullptr; //used for searching
 //   std::shared_ptr<MapPointDB> mappoint_db_global  = nullptr;

    std::shared_ptr<KeyFrameDB> keyframe_db_parent = nullptr; //local dbs are registred w/ parent to harmonize db tree structure with map tree structure
    std::shared_ptr<MapPointDB> mappoint_db_parent = nullptr;

    std::shared_ptr<KeyFrameDB> keyframe_db_local;    //local map data - can become nodes in global keyframe and mappoint dbs.
    std::shared_ptr<MapPointDB> mappoint_db_local;

    MapPointFactory MPfactory;


    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;
    long unsigned int firstKFid = 0;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    //multimap data
    std::vector<std::unique_ptr<Map>> sub_maps;
    std::vector<cv::Mat> sub_map_Tse3; //relationship between parent map origin and submap origin
    Map* parent_map = nullptr;

    bool registered = false;    //is this map registered to the parent or global map
    bool active = false;

    //private KeyFrame functions
    bool _addKeyFrame_(KeyFrame* pKF);
    bool _eraseKeyFrame_(KeyFrame* pKF);
    bool _isKFErasable_(KeyFrame* pKF, bool &erasble);

    //private mappointDB functions
    bool _addMapPoint_(MapPoint* pMP, KeyFrame* pKF_ref, int idx);
    bool _eraseMapPoint_(MapPoint* pMP);

    bool _addAssociation_(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace);
    bool _eraseAssociation_(KeyFrame* pKF,  MapPoint* pMP);

};

} //namespace ORB_SLAM

#endif // MAP_H
