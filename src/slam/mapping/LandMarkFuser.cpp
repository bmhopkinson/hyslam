#include <LandMarkFuser.h>
#include <MapPoint.h>
#include <FeatureMatcher.h>
#include <KeyFrameDB.h>
#include <Camera.h>

#include <vector>
#include <set>
#include <memory>

namespace HYSLAM{
LandMarkFuser::LandMarkFuser(KeyFrame* pKF_, Map* pMap_,LandMarkFuserParameters params_, FeatureFactory* factory, std::ofstream &log_) :
pKF(pKF_), pMap(pMap_), params(params_), feature_factory(factory)
{
    log = &log_;
}

void LandMarkFuser::run(){

    int n_total_fused = 0;
    const Camera camera = pKF->getCamera();

    bool is_mono = camera.sensor == 0;
    int nn = params.N_neighborKFs_stereo;
    if(is_mono)
        nn= params.N_neighborKFs_mono;

    //neighbors of focal KF will be used for fusing
    const std::vector<KeyFrame*> vpNeighKFs = pMap->getBestCovisibilityKeyFrames(pKF, nn);
    std::set<KeyFrame*> KF_fuse_targets;
    for(std::vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad())
            continue;
        KF_fuse_targets.insert(pKFi);

        // include neighbors of neighbors
        const std::vector<KeyFrame*> vpSecondNeighKFs = pMap->getBestCovisibilityKeyFrames(pKFi, params.N_secondNeighbors);
        for(std::vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnId==pKF->mnId)
                continue;
            KF_fuse_targets.insert(pKFi2);
        }
    }

    // Search matches by projection from current KF in target KFs
   // FeatureMatcher matcher;
   std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();
    std::vector<MapPoint*> vpMapPointMatches = pKF->GetMapPointMatches();
    for(std::set<KeyFrame*>::iterator vit=KF_fuse_targets.begin(), vend=KF_fuse_targets.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
    //    std::map<std::size_t, MapPoint*> fuse_matches;
      //  matcher.Fuse(pKFi,vpMapPointMatches, fuse_matches); //find fuse matches

        std::map<std::size_t, MapPoint*> fuse_matches;
        matcher->Fuse(pKFi,vpMapPointMatches, fuse_matches); //find fuse matches
        n_total_fused = fuse_mappoints(pKFi,  fuse_matches);
    }

    // Search matches by projection from target KFs in current KF
    std::set<MapPoint*> landmark_fuse_candidates;

    for(std::set<KeyFrame*>::iterator vitKF=KF_fuse_targets.begin(), vend=KF_fuse_targets.end(); vitKF!=vend; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        std::vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(std::vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad())
                continue;
            landmark_fuse_candidates.insert(pMP);
        }
    }

    std::vector<MapPoint*> vlandmark_fuse_candidates(landmark_fuse_candidates.begin(), landmark_fuse_candidates.end());
    std::map<std::size_t, MapPoint*> fuse_matches;
    matcher->Fuse(pKF,vlandmark_fuse_candidates, fuse_matches);  //would like to move the actual fusing out of the matchers - have matcher return landmarks for fusing and fuse here
    n_total_fused += fuse_mappoints(pKF,  fuse_matches);
    // std::cout <<"finished Fuse in SearchInNeighbors for KF:" << mpCurrentKeyFrame->mnId << std::endl;

    // Update connections in covisibility graph
    pMap->update(pKF);

    *log << "fused_mpts: " << n_total_fused  <<"\t";
    has_finished = true;
}


int LandMarkFuser::fuse_mappoints(KeyFrame* pKFfuse,  std::map<std::size_t, MapPoint*> &fuse_matches){
    //merge fuse matches
    int n_success=0;
    for(auto it = fuse_matches.begin(); it != fuse_matches.end(); ++it){
        size_t idx = it->first;
        MapPoint* lm_fuse = it->second;
        MapPoint* lm_current = pKFfuse->hasAssociation(idx);
        if(lm_current)
        {
            if(!lm_current->isBad())
            {
                //std::cout << "Fusing mappoint" << std::endl;
                if(lm_current->Observations() > lm_fuse->Observations()){

                    int res = pMap->replaceMapPoint(lm_fuse, lm_current);
                    if(res != 0)
                    {
                      //  std::cout << "couldn't fuse: " << lm_fuse->mnId << ", " << lm_current->mnId <<std::endl;
                    }
                    else{
                        n_success++;
                    }

                }
                else{
                    int res =  pMap->replaceMapPoint(lm_current, lm_fuse);
                    if(res != 0){
                       // std::cout << "couldn't fuse: " << lm_fuse->mnId << ", " << lm_current->mnId <<std::endl;
                    }
                    else{
                        n_success++;
                    }
                }
            }
        }
        else
        {
            pMap->addAssociation(pKFfuse, idx, lm_fuse, true);
            n_success++;
        }
    }
    return n_success;
}

}//end namespace