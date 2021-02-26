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

#include <KeyFrameDB.h>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"



namespace HYSLAM
{

KeyFrameDB::KeyFrameDB(){}

void KeyFrameDB::setVocab(ORBVocabulary* pVoc)
{
    place_recog.setVocab(pVoc);
}


void KeyFrameDB::add(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(kfdb_mutex);
    KF_set.insert(pKF);
    int res = covis_graph.addNode(pKF);
    if(res == -1){ std::cout << "rejected from covis graph, pKF: " << pKF->mnId <<std::endl;}

     std::vector<KeyFrame*> parent = covis_graph.GetBestCovisibilityKeyFrames(pKF, 1);
     if(!parent.empty()) {  //will be empty for keyframe 0
         spanning_tree.addNode(pKF, parent[0]);
     } else {
        spanning_tree.addNode(pKF, nullptr);
     }

     place_recog.add(pKF);
}

void KeyFrameDB::erase(KeyFrame* pKF, std::string option)
{
    if(option == "All" ){
        std::unique_lock<std::mutex> lock( kfdb_mutex);
        //set bad and set mTcp which is an incremental motion between parent and KF
        pKF->mbBad = true;
        KeyFrame* mpParent = spanning_tree.getParent(pKF);
        pKF->mTcp = pKF->GetPose()*mpParent->GetPoseInverse();  //would be good to eliminate the need to do this

        updateSpanningTreeforKeyFrameRemoval(pKF);
        spanning_tree.eraseNode(pKF);
        KF_set.erase(pKF);
        covis_graph.eraseNode(pKF);
        //  place_recog.erase(pKF);  //for consistency w/ ORB_SLAM implementation - on the other hand i dont' see any point in keeping bad keyframes in the place recognition candiate list


    }
    else if(option == "Covis"){
        covis_graph.eraseNode(pKF);
    }
    else {
        std::cout << "KeyFrameDB::erase option: " << option << " does not exist"  << std::endl;
    }

}


void KeyFrameDB::update(KeyFrame* pKF){
    covis_graph.UpdateConnections(pKF);
}

bool KeyFrameDB::updateSpanningTreeforKeyFrameRemoval(KeyFrame* pKF){
    // Update Spanning Tree - requires cooperation of spanning tree and covis graph so can't be done at a lower level (e.g. within spanning_tree);
    KeyFrame* mpParent = spanning_tree.getParent(pKF);
//    std::cout << "KF: " << pKF << " id: " << pKF->mnId <<   "parent: " << mpParent <<std::endl;
//    std::cout <<" parent Id: "  << mpParent->mnId << std::endl;
    std::set<KeyFrame*> sParentCandidates;
    sParentCandidates.insert(mpParent);

    std::set<KeyFrame*> children = spanning_tree.getChildren(pKF);
    while(!children.empty() ) { //reassign children to new parents
        bool bContinue = false;

        int max = -1;
        KeyFrame* pC;
        KeyFrame* pP;

        for(auto sit = children.begin(); sit!=children.end(); sit++)
        {
            KeyFrame* pKF_child = *sit;
            if( pKF_child->isBad())
                continue;

            // Check if a parent candidate is connected to the keyframe
            std::vector<KeyFrame*> vpConnected = covis_graph.GetVectorCovisibleKeyFrames( pKF_child);
            for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
            {
                for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                {
                    if(vpConnected[i]->mnId == (*spcit)->mnId)
                    {
                        int w =  covis_graph.GetWeight(pKF_child, vpConnected[i]);
                        if(w>max)
                        {
                            pC =  pKF_child;
                            pP = vpConnected[i];
                            max = w;
                            bContinue = true;
                        }
                    }
                }
            }
        }

        if(bContinue)
        {
            spanning_tree.changeParent(pC, pP);
            sParentCandidates.insert(pC);
            children.erase(pC);
        }
        else
            break;
    }

    // If a child has no covisibility links with any parent candidate, assign to the original parent of this KF
    if(children.empty()) {
        for (set<KeyFrame *>::iterator sit =children.begin(); sit != children.end(); sit++) {
            spanning_tree.changeParent(*sit, mpParent);
        }
    }

    return true;
}


void KeyFrameDB::clear()
{
    KF_set.clear();
    covis_graph.clear();
    spanning_tree.clear();
    place_recog.clear();
}
/*
void KeyFrameDB::validateCovisiblityGraph(){
    covis_graph.validateCovisiblityGraph();
}
*/
std::set<KeyFrame*> KeyFrameDB::GetConnectedKeyFrames(KeyFrame* pKF){
    return covis_graph.GetConnectedKeyFrames(pKF);
}

std::vector<KeyFrame* > KeyFrameDB::GetVectorCovisibleKeyFrames(KeyFrame* pKF){
    return covis_graph.GetVectorCovisibleKeyFrames(pKF);
}

std::vector<KeyFrame*> KeyFrameDB::GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N){
    return covis_graph.GetBestCovisibilityKeyFrames(pKF, N);
}

std::vector<KeyFrame*> KeyFrameDB::GetCovisiblesByWeight(KeyFrame* pKF, const int &w){
    return covis_graph.GetCovisiblesByWeight(pKF, w);
}

int KeyFrameDB::GetWeight(KeyFrame* pKFnode, KeyFrame* pKFquery){
    return covis_graph.GetWeight(pKFnode, pKFquery);
}
/*
void KeyFrameDB::validateSpanningTree(){
    spanning_tree.validateSpanningTree();
}
*/
std::set<KeyFrame*> KeyFrameDB::getChildren(KeyFrame* pKF) {  // used in LoopClosing and Tracking
    return spanning_tree.getChildren(pKF);
}

KeyFrame* KeyFrameDB::getParent(KeyFrame* pKF) {
    return spanning_tree.getParent(pKF);
}

bool KeyFrameDB::isChild(KeyFrame* pKF_node, KeyFrame* pKF_query){
    return spanning_tree.isChild(pKF_node, pKF_query);

}

vector<KeyFrame*> KeyFrameDB::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
    if (covis_graph.inGraph(pKF)){
      std::set<KeyFrame *> KFs_excluded = covis_graph.GetConnectedKeyFrames(pKF);
      KFs_excluded.insert(pKF);
      return place_recog.detectLoopCandidates(pKF, minScore, KFs_excluded);
    } else {
        return vector<KeyFrame*>();
    }
}

vector<KeyFrame*> KeyFrameDB::DetectRelocalizationCandidates(Frame *F)
{
   return place_recog.detectRelocalizationCandidates(F);
}

} //namespace ORB_SLAM
