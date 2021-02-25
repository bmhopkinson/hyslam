#ifndef LANDMARKMATCHES_H_
#define LANDMARKMATCHES_H_

#include <map>
#include <vector>
#include <MapPoint.h>

namespace ORB_SLAM2{

//class MapPoint;  //not sure why i need to do this but i do - even though i'm including MapPoint.h

struct LandMarkMatches{
    using LandMarkMatches_t = std::map<int, MapPoint*>;

    //data
    LandMarkMatches_t views_to_landmarks;
    std::map<int, bool> outliers;  //all associations start as inliers but can later be labeled as an outlier
    std::map<MapPoint*, int> n_frames_tracked; // number of consecutive frames the mappoint has been tracked in
    int n_matches = 0;


    //functions
    int total_matches() const {return n_matches;} // all matches including outliers
    int numValidMatches() const; //matches that aren't outliers
    //determine if landmark i is assocaited with a mappoint, if so return it, otherwise return nullptr
    MapPoint* hasAssociation(int i) const;
    int hasAssociation(MapPoint* pMP) const;

    //associate Landmark i with mappoint pMP; if "replace" allows replacement of previously matched maptpoint, return 0 if succesful
    int associateLandMark(int i, MapPoint* pMP, bool replace);

    // remove mappoint association with KeyPoint i or MapPoint pMP
    int removeLandMarkAssociation(int i);
    int removeLandMarkAssociation(MapPoint* pMP);

    //clear all mappoint to keypoint associations
    int clearAssociations();

    //outlier functions
    bool isOutlier(int i) const ;
    int  setOutlier(int i, bool is_outlier);

    //tracking data
    void propagateTracking(const LandMarkMatches &matches_previous);
    int getNumFramesTracked(MapPoint* pMP) const;

    //iterator functionality
    using iterator = LandMarkMatches_t::iterator;
    using const_iterator = LandMarkMatches_t::const_iterator;
    iterator begin() {return views_to_landmarks.begin(); }
    iterator end() {return views_to_landmarks.end(); }
    const_iterator begin() const {return views_to_landmarks.begin(); }
    const_iterator end() const {return views_to_landmarks.end(); }
    const_iterator cbegin() const {return views_to_landmarks.cbegin(); }
    const_iterator cend() const {return views_to_landmarks.cend(); }

};

}

#endif
