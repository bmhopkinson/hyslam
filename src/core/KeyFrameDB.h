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

/*
 * holds all the information about relationships between keyframes:
 *   CovisibilityGraph
 *   SpanningTree
 *   PlaceRecognizer
 *
 * KeyFrameDB is a recursive tree structure to allow sharing (or not) of information between submaps (see Map). function calls to the main db are implemented recursively to search any child KeyFrameDBs.
 *   The component data structures (CovisibilityGraph, SpanningTree, PlaceRecognizer) are NOT recurvise data structures so managing these in the context of the recurvise structure of the KeyFrameDB requires some work.
 *
 *   mostly just passes function calls on to these underlying objects.
 *   the only real work it does is reconfiguring the spanning tree when a keyframe is erased b/c doing so requires cooperation between CovisibilityGraph and SpanningTree
 *   working functions:
 *
 *   erase(KeyFrame*) - awkward inferace where a string is passed depending on what you want to erase:
 *      - "All": sets KF bad, updates spanning tree (calling updateSpanningTreeforKeyFrameRemoval() ), then erases KF in CovisibilityGraph, SpanningTree and KeyFrameDB
 *      - "Covis": only erases KF from CovisibililityGraph - doesn't do anything to spanning tree or keyframeDB
 *
 *   updateSpanningTreeforKeyFrameRemoval(KeyFrame* pKF ) - assigns new parents to children of pKF, starts with parent of pKF tries to assign children to this if covisible - add more candidates (children) if needed
 *
 *
 */

//SHOULD TAKE OVER REsponsiblity of adding assocations to keyframe, track these, and periodically update covis,etc based on assocaition changes (e.g. after 10 changes do update).

#include <KeyFrame.h>
#include <Frame.h>
#include <FeatureVocabulary.h>
#include <CovisibilityGraph.h>
#include <SpanningTree.h>
#include <PlaceRecognizer.h>

#include <vector>
#include <list>
#include <set>
#include <mutex>
#include <string>
#include <memory>

namespace HYSLAM
{

class KeyFrame;
class Frame;

class KeyFrameDB
{
public:
    KeyFrameDB();

    KeyFrameDB(FeatureVocabulary &voc);

    void setVocab(FeatureVocabulary* pVoc);

    FeatureVocabulary* getVocab();

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF, std::string option); //i'm sure there's a better way to do this

    bool exists(KeyFrame *pKF);

    bool update(KeyFrame* pKF);

    bool updateSpanningTreeforKeyFrameRemoval(KeyFrame* pKF);

    std::set<KeyFrame*> getAllKeyFrames();//{return KF_set;};

    long unsigned int getNumberOfKeyFrames(){return KF_set.size();};

    void clear();

    void addChild(std::shared_ptr<KeyFrameDB> child);
    void removeChild(std::shared_ptr<KeyFrameDB> child);

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
    std::set<KeyFrame*> getSpanningTreeChildren(KeyFrame* pKF);  // used in LoopClosing and Tracking
    KeyFrame* getSpanningTreeParent(KeyFrame* pKF);
    bool changeSpanningTreeParent(KeyFrame* pKF_node, KeyFrame* pKF_newparent);
    bool isChild(KeyFrame* pKF_node, KeyFrame* pKF_query); //used in Optimizer
    bool eraseSpanningTreeNode(KeyFrame* pKF);

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

    std::list<std::shared_ptr<KeyFrameDB>> sub_dbs;

    //private implementations
    bool _erase_(KeyFrame* pKF, std::string option);
    bool _erase_connections_(KeyFrame* pKF, std::set<KeyFrame*> &pKF_conn);
    bool _eraseKFDBset_(KeyFrame* pKF);

    bool _getSpanningTreeChildren_(KeyFrame* pKF, std::set<KeyFrame*> &children);
    bool _getSpanningTreeParent_(KeyFrame* pKF_node, KeyFrame* &pKF_parent);
    bool _changeSpanningTreeParent_(KeyFrame *pKF_node, KeyFrame *pKF_newparent, int &result);
    bool _isChild_(KeyFrame* pKF_node, KeyFrame* pKF_query, bool &result);
    bool _eraseSpanningTreeNode_(KeyFrame* pKF, bool &result);

    bool _eraseCovisGraph_(KeyFrame* pKF);

};

} //namespace ORB_SLAM

#endif
