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

#ifndef KEYFRAMEDB_H
#define KEYFRAMEDB_H

//SHOULD TAKE OVER REsponsiblity of adding assocations to keyframe, track these, and periodically update covis,etc based on assocaition changes (e.g. after 10 changes do update).

#include <KeyFrame.h>
#include <Frame.h>
#include <ORBVocabulary.h>
#include <CovisibilityGraph.h>
#include <SpanningTree.h>
#include <PlaceRecognizer.h>

#include <vector>
#include <list>
#include <set>
#include <mutex>
#include <string>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

class KeyFrameDB
{
public:
    KeyFrameDB();

    KeyFrameDB( ORBVocabulary &voc);

   void setVocab(ORBVocabulary* pVoc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF, std::string option); //i'm sure there's a better way to do this

   void update(KeyFrame* pKF);

   bool updateSpanningTreeforKeyFrameRemoval(KeyFrame* pKF);

   void clear();

   //covisibility graph functions
  //  void validateCovisiblityGraph();
    CovisibilityGraph& getCovisibilityGraph() {return covis_graph;}
    std::set<KeyFrame*> GetConnectedKeyFrames(KeyFrame* pKF); //used in LoopClosing and KeyFrameDatabase
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames(KeyFrame* pKF); //used in LocalMapping, LocalBA, and LoopClosing
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N); //used in Tracking, KeyFrameDatabase, LocalMapping
    std::vector<KeyFrame*> GetCovisiblesByWeight(KeyFrame* pKF, const int &w); // used in Optimizer, MapDrawer
    int GetWeight(KeyFrame* pKFnode, KeyFrame* pKFquery); //using in Optimizer and KeyFrame implementation

    //spanning tree functions
   // void validateSpanningTree();
    std::set<KeyFrame*> getChildren(KeyFrame* pKF);  // used in LoopClosing and Tracking
    KeyFrame* getParent(KeyFrame* pKF); // used in Tracking, Optimizer, System, Trajectory, LocalMapping, MapDrawer - used whenever a KeyFrame is bad to work back tree
    bool isChild(KeyFrame* pKF_node, KeyFrame* pKF_query); //used in Optimizer

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:
    std::set<KeyFrame*> KF_set;

    CovisibilityGraph covis_graph;

    SpanningTree spanning_tree;

    PlaceRecognizer place_recog;

  // Mutex
  std::mutex kfdb_mutex;
};

} //namespace ORB_SLAM

#endif
