#include <TrackLocalMap.h>
#include <ORBmatcher.h>
#include <Optimizer.h>

namespace ORB_SLAM2 {
TrackLocalMap::TrackLocalMap(optInfo optimizer_info_,  const TrackLocalMapParameters &params_ )
: optimizer_info(optimizer_info_), params(params_) {}

int TrackLocalMap::track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap_, Trajectory* trajectory){

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    pcurrent_frame = &current_frame;
    pMap = pMap_;

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&current_frame, optimizer_info);
    int mnMatchesInliers = 0;

    const LandMarkMatches matches = current_frame.getLandMarkMatches();
    for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
        int LMid = it->first;
        MapPoint* pMP = it->second;
        if(!pMP){continue;}
        if(!current_frame.isOutlier(LMid)){
            if(pMP->Observations()>0){
                mnMatchesInliers++;
            }

        } else if (current_frame.getCamera().sensor ==1){ //1 = stereo
            current_frame.removeLandMarkAssociation(LMid);
        }
    }

    //   std::cout << "TrackLocalMap: num mps to track: " << local_map_points.size()
     //      << "\t inliers: " << mnMatchesInliers << std::endl;

    return mnMatchesInliers;

}

void TrackLocalMap::UpdateLocalMap(){
    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

    if(pcurrent_frame->getCamera().camName == "SLAM"){
        std::vector<MapPoint*> v_lmp(local_map_points.begin(), local_map_points.end());
        pMap->SetReferenceMapPoints(v_lmp);// This is for visualization
    }

}

void TrackLocalMap::SearchLocalPoints(){
    const LandMarkMatches matches = pcurrent_frame->getLandMarkMatches();
    for (auto it = matches.cbegin(); it != matches.cend(); ++it) {
        int LMid = it->first;
        MapPoint *pMP = it->second;
        if(!pMP){continue;}
        if (pMP->isBad()) {
            pcurrent_frame->removeLandMarkAssociation(LMid);
        } else {
            pMP->mnLastFrameSeen = pcurrent_frame->mnId;
            local_map_points.erase(pMP);
        }
    }

    ORBmatcher matcher(params.match_nnratio);
    //int th = 5;
    std::vector<MapPoint*> v_lmp(local_map_points.begin(), local_map_points.end());
    int n_matches = matcher.SearchByProjection(*pcurrent_frame, v_lmp, params.match_radius_threshold);

//    std::cout << "tracklocal map, N_local_mpts: " << v_lmp.size() << ", matches SearchByProjection: "  << n_matches << std::endl;

}

void TrackLocalMap::UpdateLocalKeyFrames(){
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    const LandMarkMatches matches = pcurrent_frame->getLandMarkMatches();
    for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
        int LMid = it->first;
        MapPoint *pMP = it->second;
        if(!pMP){continue;}
        if(!pMP->isBad())
        {
            const map<KeyFrame*,size_t> observations = pMP->GetObservations();
            for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                keyframeCounter[it->first]++;
        }
        else
        {
            pcurrent_frame->removeLandMarkAssociation(LMid);
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    local_key_frames.clear();

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        local_key_frames.insert(pKF);
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(std::set<KeyFrame*>::const_iterator itKF=local_key_frames.begin(), itEndKF=local_key_frames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        //if(local_key_frames.size()>80)
        if(local_key_frames.size()>params.N_max_local_keyframes) {
            break;
        }

        KeyFrame* pKF = *itKF;

       // const vector<KeyFrame*> vNeighs = pMap->getKeyFrameDB()->GetBestCovisibilityKeyFrames(pKF, 10);
        const vector<KeyFrame*> vNeighs = pMap->getKeyFrameDB()->GetBestCovisibilityKeyFrames(pKF, params.N_neighbor_keyframes);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                local_key_frames.insert(pNeighKF);
                break;
            }
        }

        //can probably eliminate inclusion of children and parents  - seems like overkill and shouldn't they be in the covis graph if they're useful
        //const set<KeyFrame*> spChilds = pKF->GetChilds();
        const set<KeyFrame*> spChilds =  pMap->getKeyFrameDB()->getChildren(pKF);
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                local_key_frames.insert(pChildKF);
                break;
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            local_key_frames.insert(pParent);
            break;
        }

    }

  //  std::cout << "TrackLocalMap num local KFs: " << local_key_frames.size() << std::endl;

    if(pKFmax)
    {
   //     std::cout << "TrackLocalMap: frame: " << pcurrent_frame->mnId << " , max KF: " << pKFmax->mnId << std::endl;
    //    mpReferenceKF[cam_cur] = pKFmax;  MAKE SURE TO SET THIS IN MAIN TRACKING OBJECT!!!!!!!!!!!!!!!!
     //   pcurrent_frame->mpReferenceKF = pKFmax;
    }
}

void TrackLocalMap::UpdateLocalPoints(){
    local_map_points.clear();

    for(std::set<KeyFrame*>::const_iterator itKF=local_key_frames.begin(), itEndKF=local_key_frames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const std::vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(std::vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP){continue;}
            if(!pMP->isBad())
            {
                local_map_points.insert(pMP);
            }
        }
    }
}


}//end namespace
