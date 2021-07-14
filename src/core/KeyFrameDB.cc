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

     KeyFrame* parent = spanning_tree.parentForNewKeyFrame();
     spanning_tree.addNode(pKF, parent);
  //   if(parent) {  //will be empty for keyframe 0
  //       spanning_tree.addNode(pKF, parent);
  //   } else {
  //      spanning_tree.addNode(pKF, nullptr);
  //   }

     place_recog.add(pKF);
}

void KeyFrameDB::erase(KeyFrame* pKF, std::string option)
{
    std::set<KeyFrame*> pKF_conn = GetConnectedKeyFrames(pKF);
    std::set<KeyFrame*> difference;  // in other KeyFrameDBs - _erase_ takes care of erasing connections to KFs in this covis_graph
    std::set_difference(pKF_conn.begin(), pKF_conn.end(), KF_set.begin(), KF_set.end(), std::inserter(difference, difference.begin())); //finds KFs in pKF_conn that are NOT in KF_set
    _erase_(pKF, option);
    _erase_connections_(pKF, difference);

}


bool KeyFrameDB::_erase_(KeyFrame *pKF, std::string option) {
    if(exists(pKF)) {
        if (option == "All") {
            std::unique_lock<std::mutex> lock(kfdb_mutex);
            //set bad and set mTcp which is an incremental motion between parent and KF
            pKF->mbBad = true;
            //KeyFrame *mpParent = spanning_tree.getParent(pKF);
            KeyFrame *mpParent = getSpanningTreeParent(pKF);
            if (mpParent) {
                pKF->mTcp = pKF->GetPose() * mpParent->GetPoseInverse();  //would be good to eliminate the need to do this
            }

            //       std::cout << "updateSpanningTreeforKeyFrameRemoval" << std::endl;
            updateSpanningTreeforKeyFrameRemoval(pKF);
            eraseSpanningTreeNode(pKF);
            _eraseKFDBset_(pKF);

            _eraseCovisGraph_(pKF);
            //  place_recog.erase(pKF);  //for consistency w/ ORB_SLAM implementation - on the other hand i dont' see any point in keeping bad keyframes in the place recognition candiate list
        } else if (option == "Covis") {
            _eraseCovisGraph_(pKF);
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


bool KeyFrameDB::_eraseKFDBset_(KeyFrame *pKF) {
    if(KF_set.count(pKF)) {
        KF_set.erase(pKF);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_eraseKFDBset_(pKF)){
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
    // BH 2021/07/08 simplified this and now simply assign parent of keyframe being removed to current children
    KeyFrame* mpParent = getSpanningTreeParent(pKF);
    std::set<KeyFrame*> children = getSpanningTreeChildren(pKF);
    std::set<KeyFrame*> sParentCandidates;
    if(mpParent) {
        for (std::set<KeyFrame *>::iterator sit =children.begin(); sit != children.end(); sit++) {
            changeSpanningTreeParent(*sit, mpParent);
        }
    }
    return true;

    /*
        sParentCandidates.insert(mpParent);
    }

    std::set<KeyFrame*> children = spanning_tree.getSpanningTreeChildren(pKF);
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
*/

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
std::set<KeyFrame *> KeyFrameDB::getSpanningTreeChildren(KeyFrame *pKF) {
    std::set<KeyFrame *> children;
    if(_getSpanningTreeChildren_(pKF, children)){
        return children;
    }
    return std::set<KeyFrame *>();
}

 bool KeyFrameDB::_getSpanningTreeChildren_(KeyFrame* pKF,std::set<KeyFrame*> &children ) {  // used in LoopClosing and Tracking
    if(spanning_tree.exists(pKF)) {
        children = spanning_tree.getChildren(pKF);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_getSpanningTreeChildren_(pKF, children)){
                return true;
            }
        }
    }
    return false;
}

KeyFrame* KeyFrameDB::getSpanningTreeParent(KeyFrame* pKF) {
    KeyFrame* parent  = nullptr;
    if(_getSpanningTreeParent_(pKF, parent)){
        return parent;
    }
    return nullptr;
}

bool KeyFrameDB::_getSpanningTreeParent_(KeyFrame *pKF_node, KeyFrame* &pKF_parent) {
    if(spanning_tree.exists(pKF_node)) {
        pKF_parent = spanning_tree.getParent(pKF_node);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_getSpanningTreeParent_(pKF_node, pKF_parent)){
                return true;
            }
        }
    }
    return false;
}

bool KeyFrameDB::changeSpanningTreeParent(KeyFrame *pKF_node, KeyFrame *pKF_newparent) {
    int result;
    if(_changeSpanningTreeParent_(pKF_node, pKF_newparent, result)) {
        if(result ==0 ) {
            return true;
        } {
            return false;
        }
    }
    return false;

}

bool KeyFrameDB::_changeSpanningTreeParent_(KeyFrame *pKF_node, KeyFrame *pKF_newparent, int &result) {

    if(spanning_tree.exists(pKF_node)) {
        result = spanning_tree.changeParent(pKF_node, pKF_newparent);
        return true; //indicates spanning tree with pKF_node has been found
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_changeSpanningTreeParent_(pKF_node, pKF_newparent, result)){
                return true;
            }
        }
    }
    return false;
}

bool KeyFrameDB::isChild(KeyFrame* pKF_node, KeyFrame* pKF_query){
   bool result;
   if(_isChild_(pKF_node, pKF_query, result)){
       return result;
   }
   return false;
}

bool KeyFrameDB::_isChild_(KeyFrame *pKF_node, KeyFrame *pKF_query, bool &result) {
    if(spanning_tree.exists(pKF_node)) {
        result =  spanning_tree.isChild(pKF_node, pKF_query);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_isChild_(pKF_node, pKF_query, result)){
                return true;
            }
        }
    }
    return false;
}

bool KeyFrameDB::eraseSpanningTreeNode(KeyFrame *pKF) {
    bool result;
    if(_eraseSpanningTreeNode_(pKF, result)){
        return result;
    }
    return false;
}


bool KeyFrameDB::_eraseSpanningTreeNode_(KeyFrame *pKF, bool &result) {
    if(spanning_tree.exists(pKF)) {
        result = spanning_tree.eraseNode(pKF);
        return true; //indicates spanning tree containing pKF found
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_eraseSpanningTreeNode_(pKF, result)){
                return true;
            }
        }
    }
    return false;  //not ideal - perhaps return an int and return -1 if pKF_node not found
}

bool KeyFrameDB::_eraseCovisGraph_(KeyFrame *pKF) {
    if(covis_graph.inGraph(pKF)){
        covis_graph.eraseNode(pKF);
        return true;
    } else {
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            if((*it)->_eraseCovisGraph_(pKF)){
                return true;
            }
        }
    }
    return false;
}

std::vector<KeyFrame*> KeyFrameDB::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
    //NOTE MAY INSTEAD WANT TO RETURN scores with candidates and screen further for highest scores as the place recognizer uses some relative scoring metrics
    std::vector<KeyFrame*> candidates =  place_recog.detectLoopCandidates(pKF, minScore);

    for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
        std::vector<KeyFrame*> candidates_addn = (*it)->DetectLoopCandidates(pKF, minScore);
        if(!candidates_addn.empty()){
            candidates.insert(candidates.end(), candidates_addn.begin(), candidates_addn.end());
        }
    }
    return candidates;
}

std::set<KeyFrame*> KeyFrameDB::DetectRelocalizationCandidates(Frame *F)
{
    //NOTE MAY INSTEAD WANT TO RETURN scores with candidates and screen further for highest scores as the place recognizer uses some relative scoring metrics
    std::vector<KeyFrame*> candidates_vec =  place_recog.detectRelocalizationCandidates(F);
    std::set<KeyFrame*> candidates(candidates_vec.begin(), candidates_vec.end());

    for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
        std::set<KeyFrame*> candidates_addn = (*it)->DetectRelocalizationCandidates(F);
        if(!candidates_addn.empty()){
            candidates.insert( candidates_addn.begin(), candidates_addn.end());
        }
    }
    return candidates;

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

std::set<KeyFrame *> KeyFrameDB::getAllKeyFrames() {
     std::set<KeyFrame *> all_keyframes = KF_set;
     for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
         std::set<KeyFrame *> addn_keyframes = (*it)->getAllKeyFrames();
         all_keyframes.insert(addn_keyframes.begin(), addn_keyframes.end());
     }
     return all_keyframes;

}

long unsigned int KeyFrameDB::getNumberOfKeyFrames() {
    long unsigned int n_kfs = KF_set.size();
    for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
        n_kfs+= (*it)->getNumberOfKeyFrames();
    }
    return n_kfs;
}


} //namespace ORB_SLAM
