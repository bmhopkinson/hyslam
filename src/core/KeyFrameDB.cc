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
#include <DBoW2/BowVector.h>



namespace HYSLAM
{

KeyFrameDB::KeyFrameDB(){}

void KeyFrameDB::setVocab(FeatureVocabulary* pVoc)
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
    std::set<KeyFrame*> pKF_conn = covis_graph.GetConnectedKeyFrames(pKF);
    std::set<KeyFrame*> difference;  // in other KeyFrameDBs - _erase_ takes care of erasing connections to KFs in this covis_graph
    std::set_difference(pKF_conn.begin(), pKF_conn.end(), KF_set.begin(), KF_set.end(), std::back_inserter(difference)); //finds KFs in pKF_conn that are NOT in KF_set
    _erase_(pKF, option);
    _erase_connections_(pKF, difference);

}


bool KeyFrameDB::_erase_(KeyFrame *pKF, std::string option) {
    if(KF_set.count(pKF)) {
        if (option == "All") {
            std::unique_lock<std::mutex> lock(kfdb_mutex);
            //set bad and set mTcp which is an incremental motion between parent and KF
            pKF->mbBad = true;
            KeyFrame *mpParent = spanning_tree.getParent(pKF);
            if (mpParent) {
                pKF->mTcp = pKF->GetPose() * mpParent->GetPoseInverse();  //would be good to eliminate the need to do this
            }

            //       std::cout << "updateSpanningTreeforKeyFrameRemoval" << std::endl;
            updateSpanningTreeforKeyFrameRemoval(pKF);
            spanning_tree.eraseNode(pKF);
            KF_set.erase(pKF);

            //       std::cout << "covis_graph.eraseNode" << std::endl;

            covis_graph.eraseNode(pKF);
            //  place_recog.erase(pKF);  //for consistency w/ ORB_SLAM implementation - on the other hand i dont' see any point in keeping bad keyframes in the place recognition candiate list
        } else if (option == "Covis") {
            covis_graph.eraseNode(pKF);
        } else {
            std::cout << "KeyFrameDB::erase option: " << option << " does not exist" << std::endl;
        }
        return true;
    }
    else{
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_erase_(pKF, option)){
                return true;
            }
        }
    }

    return false;
}

bool KeyFrameDB::_erase_connections_(KeyFrame *pKF, std::set<KeyFrame *> &pKF_conn) {
    for(auto it = pKF_conn.begin(); it != pKF_conn.end(); ){
        KeyFrame* pKFcur = *it;
        if(covis_graph.inGraph(pKFcur)){
            covis_graph.eraseConnection(pKFcur, pKF);
            it = pKF_conn.erase(it);
        }
        else {
            ++it;
        }
    }
    if(pKF_conn.empty()){
        return true;
    }
    else{
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_erase_connections_(pKF, pKF_conn)){
                return true;
            }
        }
    }
    return false;
}


bool KeyFrameDB::update(KeyFrame* pKF){
    //should make this handle reciprocal updates
    if(covis_graph.inGraph(pKF)){
        covis_graph.UpdateConnections(pKF);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->update(pKF)){
                return true;
            }
        }
    }
    return false;
}

bool KeyFrameDB::updateSpanningTreeforKeyFrameRemoval(KeyFrame* pKF){
    // Update Spanning Tree - requires cooperation of spanning tree and covis graph so can't be done at a lower level (e.g. within spanning_tree);
    KeyFrame* mpParent = spanning_tree.getParent(pKF);
  //  std::cout << "KF: " << pKF << " id: " << pKF->mnId <<   "parent: " << mpParent <<std::endl;
//    std::cout <<" parent Id: "  << mpParent->mnId << std::endl;
    std::set<KeyFrame*> sParentCandidates;
    if(mpParent) {
        sParentCandidates.insert(mpParent);
    }

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
                for(std::set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
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
        for (std::set<KeyFrame *>::iterator sit =children.begin(); sit != children.end(); sit++) {
           // if(mpParent) {
                spanning_tree.changeParent(*sit, mpParent);
         //   }
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

    for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
        (*it)->clear();
    }
}
/*
void KeyFrameDB::validateCovisiblityGraph(){
    covis_graph.validateCovisiblityGraph();
}
*/
std::set<KeyFrame*> KeyFrameDB::GetConnectedKeyFrames(KeyFrame* pKF){
    std::set<KeyFrame*> pKF_conn;
    if(covis_graph.inGraph(pKF)) {
        return covis_graph.GetConnectedKeyFrames(pKF);
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            pKF_conn = (*it)->GetConnectedKeyFrames(pKF);
            if(!pKF_conn.empty()){
                return pKF_conn;
            }
        }
    }
    return pKF_conn;
}

std::vector<KeyFrame* > KeyFrameDB::GetVectorCovisibleKeyFrames(KeyFrame* pKF){
    std::vector<KeyFrame* > pKF_conn;
    if(covis_graph.inGraph(pKF)) {
        return covis_graph.GetVectorCovisibleKeyFrames(pKF);
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            pKF_conn = (*it)->GetVectorCovisibleKeyFrames(pKF);
            if(!pKF_conn.empty()){
                return pKF_conn;
            }
        }
    }
    return pKF_conn;
}

std::vector<KeyFrame*> KeyFrameDB::GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N){
    std::vector<KeyFrame* > pKF_conn;
    if(covis_graph.inGraph(pKF)) {
        return covis_graph.GetBestCovisibilityKeyFrames(pKF, N);
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            pKF_conn = (*it)->GetBestCovisibilityKeyFrames(pKF, N);
            if(!pKF_conn.empty()){
                return pKF_conn;
            }
        }
    }
    return pKF_conn;
}

std::vector<KeyFrame*> KeyFrameDB::GetCovisiblesByWeight(KeyFrame* pKF, const int &w){
    std::vector<KeyFrame* > pKF_conn;
    if(covis_graph.inGraph(pKF)) {
        return covis_graph.GetCovisiblesByWeight(pKF, w);
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            pKF_conn = (*it)->GetCovisiblesByWeight(pKF, w);
            if(!pKF_conn.empty()){
                return pKF_conn;
            }
        }
    }
    return pKF_conn;
}

int KeyFrameDB::GetWeight(KeyFrame* pKFnode, KeyFrame* pKFquery){
    int weight = -1; // less than zero indicates pKFnode not found yet
    if(covis_graph.inGraph(pKFnode)) {
        return covis_graph.GetWeight(pKFnode, pKFquery);
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            weight = (*it)->GetWeight(pKFnode, pKFquery);
            if(weight >=0){
                return weight;
            }
        }
    }
    return weight;
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

std::vector<KeyFrame*> KeyFrameDB::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
    if (covis_graph.inGraph(pKF)){
      std::set<KeyFrame *> KFs_excluded = covis_graph.GetConnectedKeyFrames(pKF);
      KFs_excluded.insert(pKF);
      return place_recog.detectLoopCandidates(pKF, minScore, KFs_excluded);
    } else {
        return std::vector<KeyFrame*>();
    }
}

std::vector<KeyFrame*> KeyFrameDB::DetectRelocalizationCandidates(Frame *F)
{
   return place_recog.detectRelocalizationCandidates(F);
}

bool KeyFrameDB::exists(KeyFrame *pKF) {
    if(KF_set.count(pKF)){
        return true;
    }
    else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->exists(pKF)){
                return true;
            }
        }
    }
    return false;
}

FeatureVocabulary *KeyFrameDB::getVocab() {
    return place_recog.getVocab();
}

void KeyFrameDB::addChild(std::shared_ptr<KeyFrameDB> child) {
    sub_dbs.push_back(child);
}

void KeyFrameDB::removeChild(std::shared_ptr<KeyFrameDB> child) {
    sub_dbs.remove(child);
}



} //namespace ORB_SLAM
