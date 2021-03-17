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

#include "LoopClosing.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "FeatureMatcher.h"
#include "GlobalBundleAdjustment.h"
#include <MapPointDB.h>
#include "g2o/types/sba/Trajectory_g2o.h"

#include<mutex>
#include<thread>


namespace HYSLAM
{

LoopClosing::LoopClosing(std::map<std::string, Map*> &_maps, FeatureVocabulary* pVoc, FeatureFactory* factory, MainThreadsStatus* thread_status_, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), maps(_maps),
    feature_factory(factory), feature_vocabulary(pVoc), thread_status(thread_status_),  mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mbFixScale(bFixScale)
{
    mnCovisibilityConsistencyTh = 3;
    mpMatchedKF = NULL;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}
/*
void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}
*/
bool LoopClosing::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    //return(!mlpLoopKeyFrameQueue.empty());
    return(input_queue->size() > 0);
}

bool LoopClosing::DetectLoop()
{

    mpCurrentKF = input_queue->pop();
    curKF_cam = mpCurrentKF->camera.camName;
    // Avoid that a keyframe can be erased while it is being process by this thread
    mpCurrentKF->setProtection();
  // std::cout << "Loop Closing: Detecting on: " << mpCurrentKF->mnId << std::endl;

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF); //utlimately just clear the keyframe directly
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
  //  const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const std::vector<KeyFrame*> vpConnectedKeyFrames = maps[curKF_cam]->getKeyFrameDB()->GetVectorCovisibleKeyFrames(mpCurrentKF);
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = feature_vocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
  //  vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);
    std::vector<KeyFrame*> vpCandidateKFs = maps[curKF_cam]->getKeyFrameDB()->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
     //   mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
     //   mpCurrentKF->SetErase();
        maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF);
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    std::vector<ConsistentGroup> vCurrentConsistentGroups;
    std::vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

     //   set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        std::set<KeyFrame*> spCandidateGroup =   maps[curKF_cam]->getKeyFrameDB()->GetConnectedKeyFrames(pCandidateKF);
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            std::set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(std::set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
   // mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
     //   mpCurrentKF->SetErase();
        maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF);
        return false;
    }
    else
    {
        return true;
    }

   // mpCurrentKF->SetErase();
    maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF);
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver

    //FeatureMatcher matcher(0.75, true);
    FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
    fm_settings.nnratio = 0.75;
    fm_settings.checkOri = true;
    feature_factory->setFeatureMatcherSettings(fm_settings);
    std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();

    std::vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    std::vector<std::vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    std::vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->setProtection();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher->SearchByBoW2(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            std::vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                std::vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher->SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++) {
         //   mvpEnoughConsistentCandidates[i]->SetErase();
            maps[curKF_cam]->ClearKeyFrameProtection(mvpEnoughConsistentCandidates[i]);
        }
       // mpCurrentKF->SetErase();
        maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF);
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
   // vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> vpLoopConnectedKFs = maps[curKF_cam]->getKeyFrameDB()->GetVectorCovisibleKeyFrames(mpMatchedKF);
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(std::vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        std::vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher->SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF) {
             //   mvpEnoughConsistentCandidates[i]->SetErase();
                maps[curKF_cam]->ClearKeyFrameProtection(mvpEnoughConsistentCandidates[i]);
            }
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++) {
          //  mvpEnoughConsistentCandidates[i]->SetErase();
            maps[curKF_cam]->ClearKeyFrameProtection(mvpEnoughConsistentCandidates[i]);
        }
       // mpCurrentKF->SetErase();
        maps[curKF_cam]->ClearKeyFrameProtection(mpCurrentKF);
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    std::cout << "Loop detected!" << std::endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
   // mpLocalMapper->RequestStop();
    thread_status->mapping.stop_requested = true;

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        mbStopGBA = true;

        while(!isFinishedGBA())
            usleep(5000);

        mpThreadGBA->join();
        delete mpThreadGBA;
    }

    // Wait until Local Mapping has effectively stopped
 //   while(!mpLocalMapper->isStopped())
    while(!thread_status->mapping.is_stopped)
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    //mpCurrentKF->UpdateConnections();
    maps[curKF_cam]->getKeyFrameDB()->update(mpCurrentKF);

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
   // mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs = maps[curKF_cam]->getKeyFrameDB()->GetVectorCovisibleKeyFrames(mpCurrentKF);
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        std::unique_lock<std::mutex> lock(maps[curKF_cam]->mMutexMapUpdate);

        for(std::vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            std::vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
            //    pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
           // pKFi->UpdateConnections();
            maps[curKF_cam]->getKeyFrameDB()->update(pKFi);
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->hasAssociation(i);

                if(pCurMP){
                   // pCurMP->Replace(pLoopMP);
                    maps[curKF_cam]->replaceMapPoint(pCurMP, pLoopMP);

                }
                else
                {

                    maps[curKF_cam]->addAssociation(mpCurrentKF, i, pLoopMP, true);
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);
    
    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    std::map<KeyFrame*, std::set<KeyFrame*> > LoopConnections;

    for(std::vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
       // vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();
        std::vector<KeyFrame*> vpPreviousNeighbors = maps[curKF_cam]->getKeyFrameDB()->GetVectorCovisibleKeyFrames(pKFi);

        // Update connections. Detect new links.
       // pKFi->UpdateConnections();
        maps[curKF_cam]->getKeyFrameDB()->update(pKFi);

        //LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        LoopConnections[pKFi] =   maps[curKF_cam]->getKeyFrameDB()->GetConnectedKeyFrames(pKFi);
        for(std::vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(std::vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(maps[curKF_cam], mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new std::thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
   // mpLocalMapper->Release();
   thread_status->mapping.release = true;

    std::cout << "Loop Closed!" << std::endl;

    mLastLoopKFid = mpCurrentKF->mnId;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
  //  FeatureMatcher matcher(0.8);
    FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
    fm_settings.nnratio = 0.8;
    feature_factory->setFeatureMatcherSettings(fm_settings);
    std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        std::vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher->Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        std::unique_lock<std::mutex> lock(maps[curKF_cam]->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
               maps[curKF_cam]->replaceMapPoint(pRep, mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            std::unique_lock<std::mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
       // mlpLoopKeyFrameQueue.clear();
        while(input_queue->size() > 0){
            input_queue->pop();
        }
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    std::cout << "Starting Global Bundle Adjustment" << std::endl;
    mpTracker->optParams.GBAtype = 2; //loop closing GBA
    std::vector<KeyFrame*> vpKFfixed;
    //fix KeyFrame = 0;
    std::vector<KeyFrame*> vpAllKFs = maps[curKF_cam]->GetAllKeyFrames();
    for(std::vector<KeyFrame*>::const_iterator vit = vpAllKFs.begin(); vit != vpAllKFs.end(); ++vit){
         KeyFrame* pKFi = *vit;
         if(pKFi->mnId == 0){
              vpKFfixed.push_back(pKFi);
         }
    }

    int nIter = 20;
    bool bRobust = false;
    g2o::Trajectory traj_g2o = mpTracker->trajectories["SLAM"]->convertToG2O();
    GlobalBundleAdjustment globalBA(vpKFfixed,  nLoopKF, nIter ,  bRobust, &mbStopGBA, maps[curKF_cam], traj_g2o, mpTracker->optParams);
    globalBA.Run();

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        std::unique_lock<std::mutex> lock(mMutexGBA);


        if(!mbStopGBA)
        {
            std::cout << "Global Bundle Adjustment finished" << std::endl;
            std::cout << "Updating map ..." << std::endl;
           // mpLocalMapper->RequestStop();
           thread_status->mapping.stop_requested = true;
            // Wait until Local Mapping has effectively stopped

          //  while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            while(!thread_status->mapping.is_stopped && !thread_status->mapping.is_finished)
            {
                usleep(1000);
            }

            // Get Map Mutex
            std::unique_lock<std::mutex> lock(maps[curKF_cam]->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            std::list<KeyFrame*> lpKFtoCheck(maps[curKF_cam]->mvpKeyFrameOrigins.begin(),maps[curKF_cam]->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
               // const set<KeyFrame*> sChilds = pKF->GetChilds();
                const std::set<KeyFrame*> sChilds = maps[curKF_cam]->getKeyFrameDB()->getChildren(pKF);
                cv::Mat Twc = pKF->GetPoseInverse();
                for(std::set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)  //if this keyframe wasn't optimized in the GBA - make correction
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const std::vector<MapPoint*> vpMPs = maps[curKF_cam]->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

           // mpLocalMapper->Release();
            thread_status->mapping.release = true;

            std::cout << "Map updated!" << std::endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
