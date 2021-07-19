#include <TrackReferenceKeyFrame.h>
#include <FeatureMatcher.h>
#include <Optimizer.h>

namespace HYSLAM{
    TrackReferenceKeyFrame::TrackReferenceKeyFrame(optInfo optimizer_info_, const TrackReferenceKeyFrameParameters &params_,  FeatureFactory* factory)
    : optimizer_info(optimizer_info_), params(params_), feature_factory(factory)
    {}

    int TrackReferenceKeyFrame::track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory){
        // Compute Bag of Words vector
        current_frame.ComputeBoW();
        const Frame& last_frame = frames[0];

        // first try BoW feature match w/ reference keyframe, if successful optimize pose
        //FeatureMatcher matcher(params.match_nnratio, false);
        FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
        fm_settings.nnratio = params.match_nnratio;
        fm_settings.checkOri = false;
        feature_factory->setFeatureMatcherSettings(fm_settings);
        std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();

        std::map<size_t, MapPoint*> matches_bow;
        int nmatches = matcher->SearchByBoW(pKF,current_frame,matches_bow);

        if(nmatches< params.N_min_matches_BoW){
            std::cout << "TrackReferenceKeyFrame failed b/c SearchByBoW < 15: only found: " << nmatches <<std::endl;
            return -1;
        }

      //  current_frame.associateLandMarkVector(vpMapPointMatches, true);
        int n_successfully_associated = current_frame.associateLandMarks(matches_bow, true);
        std::cout << "TrackRef, BoW lms associated succesfully: " << n_successfully_associated << std::endl;
        current_frame.SetPose(last_frame.mTcw);

        Optimizer::PoseOptimization(&current_frame, optimizer_info);

        // Discard outliers
        int nmatchesMap = 0;
        const LandMarkMatches matches = current_frame.getLandMarkMatches();
        for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
            int LMid = it->first;
            MapPoint* pMP = it->second;
            if(!pMP){ //trying to stop occasional segfault here - nullptrs may be creeping in
                continue;
            }
            if(current_frame.isOutlier(LMid)){
                current_frame.removeLandMarkAssociation(LMid);
                nmatches--;
                current_frame.setOutlier(LMid, false);
                pMP->mnLastFrameSeen = current_frame.mnId;

            } else if (pMP->Observations() > 0)
            {
                nmatchesMap++;
            }
        }

      //  std::cout << "TrackReferenceKeyFrame after PoseOptimization nmatchesMap: " << nmatchesMap <<std::endl;
        return nmatchesMap;
    }
}