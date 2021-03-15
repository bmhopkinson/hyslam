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

#include <MapPoint.h>
#include <KeyFrame.h>
#include <FeatureVocabulary.h>
#include <KeyFrameDB.h>
#include <MapPointDB.h>
#include <MapPointFactory.h>

#include <opencv2/core/core.hpp>

#include <set>
#include <mutex>

namespace HYSLAM
{


class Map
{
public:
    Map();

    // KeyFrame functions
    void AddKeyFrame(KeyFrame* pKF);
    void EraseKeyFrame(KeyFrame* pKF);  //details on erasing KFs is complicated due to multithreading/loop closing - probably should rename these functions
    void ClearKeyFrameProtection(KeyFrame* pKF);   //used once loop closing is done w/ a keyframe to allow its removal, and to erase it if it was marked for removal during the loop closing attempt - this isn't the best - would be better to clearprotection directly on keyframe, add keyframes to be deleted to a queue for later deletion once protection is cleared
    void SetBadKeyFrame(KeyFrame* pKF);
    void setKeyFrameDBVocab(FeatureVocabulary* pVoc); // this is a hack right now to preserve old functions of keyframe db
 //   void validateCovisiblityGraph();
    KeyFrameDB* getKeyFrameDB(){return &keyframe_db;}
    long unsigned  KeyFramesInMap();
    long unsigned int GetMaxKFid();
    
    //mappointDB functions
    MapPointDB* getMapPointDB(){ return &mappoint_db; }
    void AddMapPoint(MapPoint* pMP, KeyFrame* pKF_ref, int idx);
    MapPoint* newMapPoint( const cv::Mat &Pos, KeyFrame* pKF, int idx); 
    void eraseMapPoint(MapPoint* pMP);
    int replaceMapPoint(MapPoint* pMP_old, MapPoint* pMP_new);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    long unsigned int MapPointsInMap();
    void visibleMapPoints(KeyFrame* pKFi, std::vector<MapPoint*> &visible_mpts);
    
    //Associations
    int addAssociation(KeyFrame* pKF, int idx, MapPoint* pMP, bool replace);
    int eraseAssociation(KeyFrame* pKF,  MapPoint* pMP);
 //   int eraseAssociation(KeyFrame* pKF,  int idx);  //NOT YET IMPLEMENTED


    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:

    KeyFrameDB keyframe_db;
    MapPointDB mappoint_db;
    MapPointFactory MPfactory; 

    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAP_H
