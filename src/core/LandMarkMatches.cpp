#include <LandMarkMatches.h>
#include <iostream>

namespace HYSLAM{

MapPoint*  LandMarkMatches::hasAssociation(int i) const{
    LandMarkMatches_t::const_iterator it;
    it = views_to_landmarks.find(i);
    if(it != views_to_landmarks.end()){
        return it->second;
    }
    else {
        return nullptr;
    }
}

int LandMarkMatches::hasAssociation(MapPoint* pMP) const {
    for(auto it = views_to_landmarks.cbegin(); it != views_to_landmarks.cend(); ++it ){
        if(it->second == pMP){
            return it->first;
        }
    }
    return -1;
}

int LandMarkMatches::associateLandMark(int i, MapPoint* pMP, bool replace){
    if(!pMP){
        return -1;
    }
    MapPoint* mpt_old = hasAssociation(i);
    int idx_old = hasAssociation(pMP);
    if(mpt_old != nullptr || idx_old >= 0 ){
        if(replace){
            views_to_landmarks[i] = pMP;
            outliers[i] = false;

            if(idx_old >= 0 && idx_old != i){ //if landmark was previously associated w/ another view, erase it.
                views_to_landmarks.erase(idx_old);
            }

            return 0;
        } else {
            return -1;
        }
    } else {
        views_to_landmarks.insert({i, pMP});
        outliers.insert({i, false});
        ++n_matches;
        return 0;
    }
}

int LandMarkMatches::removeLandMarkAssociation(int i){
    MapPoint* pMP = hasAssociation(i);
    if(!pMP){ //association doesn't exist
        return -1;
    }
    else{
        views_to_landmarks.erase(i);
        outliers.erase(i);
        --n_matches;
        return 0;
    }
}

int LandMarkMatches::removeLandMarkAssociation(MapPoint* pMP){
    int n = 0;
    for(auto it = views_to_landmarks.cbegin(); it != views_to_landmarks.cend(); ){
        if(it->second == pMP){
            outliers.erase(it->first);
            it = views_to_landmarks.erase(it);
            --n_matches;
            ++n;
        }
        else {
            ++it;
        }
    }
    return n;
}


int LandMarkMatches::clearAssociations(){
    views_to_landmarks.clear();
    outliers.clear();
    n_matches = 0;
    return 0;
}

bool LandMarkMatches::isOutlier(int i) const {
    std::map<int, bool>::const_iterator it;
    it = outliers.find(i);
    if(it != outliers.end()){
        return it->second;
    }
    else {
        return false;
    }
}
int  LandMarkMatches::setOutlier(int i, bool is_outlier){
    std::map<int, bool>::iterator it;
    it = outliers.find(i);
    if(it != outliers.end()){
        it->second = is_outlier;
        return 0;
    }
    else {
        return -1;
    }
}

void LandMarkMatches::propagateTracking(const LandMarkMatches &matches_previous){
    for(auto it =views_to_landmarks.cbegin(); it != views_to_landmarks.cend(); ++it){
        MapPoint* pMP = it->second;
        n_frames_tracked[pMP] = matches_previous.getNumFramesTracked(pMP) + 1;
    }
}

int LandMarkMatches::getNumFramesTracked(MapPoint* pMP) const {
    auto it = n_frames_tracked.find(pMP);
    if(it == n_frames_tracked.end()){
        return 0;
    } else {
        return it->second;
    }
}

int LandMarkMatches::numValidMatches() const {
    int n_matches = 0;
    for(auto it = views_to_landmarks.cbegin(); it != views_to_landmarks.cend(); ++it) {
        int LMid = it->first;

        if(!isOutlier(LMid))
        {
            ++n_matches;
        }
    }
    return n_matches;
}

}//close namespace
