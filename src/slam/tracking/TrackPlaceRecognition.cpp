#include <TrackPlaceRecognition.h>
#include <FeatureMatcher.h>
#include <Optimizer.h>
#include <PnPsolver.h>

namespace HYSLAM{
TrackPlaceRecognition::TrackPlaceRecognition(optInfo optimizer_info_,TrackPlaceRecognitionParameters params_, FeatureFactory* factory)
: optimizer_info(optimizer_info_), params(params_), feature_factory(factory)
{}
int TrackPlaceRecognition::track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory)
{
    // Compute Bag of Words Vector
    current_frame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    //  vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    std::vector<KeyFrame*> vpCandidateKFs = pMap->getKeyFrameDB()->DetectRelocalizationCandidates(&current_frame);
    if(vpCandidateKFs.empty())
        return -1;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
   // FeatureMatcher matcher(params.match_nnratio_1 , true);
    FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
    fm_settings.nnratio = params.match_nnratio_1;
    fm_settings.checkOri =true;
    feature_factory->setFeatureMatcherSettings(fm_settings);
    std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();

    std::vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    std::vector<std::vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    std::vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            std::map<size_t, MapPoint*> matches_bow;
            int nmatches = matcher->SearchByBoW(pKF,current_frame,matches_bow);
          //  std::cout << "relocalize: trying KF: " << pKF->mnId << " nmatches: " << nmatches << std::endl;

            std::vector<MapPoint*> vpMapPointMatches = std::vector<MapPoint*>(current_frame.N,static_cast<MapPoint*>(NULL));
            for(auto it = matches_bow.begin(); it != matches_bow.end(); ++it){
                size_t idx= it->first;
                MapPoint* lm =  it->second;
                vpMapPointMatches[idx]  = lm;
            }

            vvpMapPointMatches[i] = vpMapPointMatches; //legacy structure - update at some point

            if(nmatches<params.N_min_matches_BoW)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(current_frame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(params.ransac.probability,params.ransac.minInliers,params.ransac.maxIterations,
                                             params.ransac.minSet,params.ransac.epsilon,params.ransac.th2);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
              //  std::cout << "candidate KF detected by BoW search: " << pKF->mnId << std::endl;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    int nGood = 0;
    bool bMatch = false;
  //  FeatureMatcher matcher2(params.match_nnratio_2, true);
    fm_settings = feature_factory->getFeatureMatcherSettings();
    fm_settings.nnratio = params.match_nnratio_2;
    fm_settings.checkOri =true;
    feature_factory->setFeatureMatcherSettings(fm_settings);
    std::unique_ptr<FeatureMatcher> matcher2 = feature_factory->getFeatureMatcher();

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            std::vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(params.PnPsolver_iterations,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {

               // std::cout << "candidate KF has pose via PnP, try to optimize: " << vpCandidateKFs[i]->mnId << std::endl;
                Tcw.copyTo(current_frame.mTcw);

                std::set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        current_frame.associateLandMark(j, vvpMapPointMatches[i][j], true);
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else{
                        current_frame.removeLandMarkAssociation(j);
                    }
                }

                nGood = Optimizer::PoseOptimization(&current_frame, optimizer_info);

                if(nGood<params.N_min_matches_PoseOpt)
                    continue;

                for(int io =0; io<current_frame.N; io++){
                    if(current_frame.isOutlier(io)){
                        current_frame.removeLandMarkAssociation(io);
                    }
                }

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<params.N_min_matches_success)
                {
                    int nadditional =matcher2->SearchByProjection(current_frame,vpCandidateKFs[i],sFound,params.match_radius_threshold_1,params.ORBdist_1);
                    if(nadditional+nGood>=params.N_min_matches_success)
                    {
                      //  std::cout << "candidate KF has pose via PnP, trying second optimization : " << vpCandidateKFs[i]->mnId << std::endl;
                        nGood = Optimizer::PoseOptimization(&current_frame, optimizer_info);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        int n_close = static_cast<int>(0.5*params.N_min_matches_success);
                        if(nGood>n_close && nGood<params.N_min_matches_success)
                        {
                            sFound.clear();
                            const LandMarkMatches matches = current_frame.getLandMarkMatches();
                            for(auto it = matches.cbegin(); it != matches.cend(); ++it) {
                                MapPoint *pMP = it->second;
                                if(!pMP){continue;}
                                sFound.insert(pMP);
                            }
                            nadditional =matcher2->SearchByProjection(current_frame,vpCandidateKFs[i],sFound,params.match_radius_threshold_2,params.ORBdist_2);
                            // Final optimization
                            if(nGood+nadditional>=params.N_min_matches_success)
                            {
                                nGood = Optimizer::PoseOptimization(&current_frame, optimizer_info);

                                for(int io =0; io<current_frame.N; io++){
                                    if(current_frame.isOutlier(io)){
                                        current_frame.removeLandMarkAssociation(io);
                                    }
                                }
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=params.N_min_matches_success)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    return nGood;
/*
    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }
*/

}

}//end namespace