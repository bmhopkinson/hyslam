#include "LocalBundleAdjustment.h"
#include "OptHelpers.h"
#include <KeyFrameDB.h>
#include <MapPointDB.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d_addons/SE3_sensor_edges.h"

#include <Eigen/StdVector>
#include <opencv2/core/eigen.hpp>

#include "Converter.h"

#include <mutex>
#include <vector>
#include <math.h>
#include <limits>

namespace HYSLAM{

int LocalBundleAdjustment::ncalls = 0;  //initialize static member valiable

LocalBundleAdjustment::LocalBundleAdjustment( KeyFrame *pCentralKF_,  bool* pbStopFlag_, Map* pMap_, g2o::Trajectory &trajectory_, optInfo optParams_) :
  pCentralKF(pCentralKF_), pbStopFlag(pbStopFlag_), BundleAdjustment( pMap_, trajectory_, optParams_ )
{
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
  typedef g2o::LinearSolverEigen<MyBlockSolver::PoseMatrixType> MyLinearSolver;

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
       g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

     setOptimizer(&optimizer);  //pass to base class BundleAdjustment

}

void LocalBundleAdjustment::Run(){
    ncalls++;
    const double chi2_thresh_mono = 5.991;   //outlier thresholds
    const double chi2_thresh_stereo = 7.815;

    //collect local keyframes and mappoints
    std::list<KeyFrame*> lLocalKeyFrames = FindLocalKFs( pCentralKF );
    std::list<MapPoint*>  lLocalMapPoints  = FindLocalMapPoints( lLocalKeyFrames, pCentralKF );
    std::list<KeyFrame*> lFixedKeyFrames = FindFixedKFs( lLocalMapPoints, pCentralKF);

    std::list<Tse3Parent> submap_tiepoints = FindSubmapTiepoints(lLocalKeyFrames);
    std::set<KeyFrame*> allKFs(lLocalKeyFrames.begin(), lLocalKeyFrames.end());
    allKFs.insert(lFixedKeyFrames.begin(), lFixedKeyFrames.end());
    std::list<KeyFrame*> submap_KFs = FindAdditionalParentSubmapKFs(submap_tiepoints, allKFs ); //find any tiepoint parent KFs that are not included in the optimization and add to fixed KFS
    lFixedKeyFrames.insert(lFixedKeyFrames.end(), submap_KFs.begin(), submap_KFs.end());

    //check if there's imaging cam data in the local KeyFrames
    imaging_camera_exists = CheckForImagingCamera(lLocalKeyFrames);

    std::list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin();
    while( lit != lLocalKeyFrames.end()) {
        KeyFrame *pKFi = *lit;
        if (pKFi->mnId == 0) {   //fix keyframe =0; not sure this is necessary but preserving behavior right now
            lFixedKeyFrames.push_back(pKFi);
            lit = lLocalKeyFrames.erase(lit);
        } else {
            lit++;
        }

    }

    //set optimizer KeyFrame vertices
    SetKeyFrameVertices( lLocalKeyFrames, false);      // Set Local KeyFrame vertices
    SetKeyFrameVertices( lFixedKeyFrames, true );       // Set Fixed KeyFrame vertices

    // set optimizer unary edges (sensor data -> keyframes)
    SetIMUEdges( lLocalKeyFrames );         //IMU quaternion measurement constraint
    SetDepthEdges( lLocalKeyFrames );      // add depth constraints
    SetGPSEdges( lLocalKeyFrames ) ;  //add GPS constraints

    // Set MapPoint vertices and edges
    SetSubMapOriginEdges(submap_tiepoints);
    if(imaging_camera_exists){
      SetImagingVertices(lLocalKeyFrames);
      SetImagingEdges(lLocalKeyFrames);
    }

    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedKeyFrames.size())*lLocalMapPoints.size();
    vpEdgesMono.reserve(nExpectedSize);
    vpEdgeKFMono.reserve(nExpectedSize);
    vpMapPointEdgeMono.reserve(nExpectedSize);
    vpEdgesStereo.reserve(nExpectedSize);
    vpEdgeKFStereo.reserve(nExpectedSize);
    vpMapPointEdgeStereo.reserve(nExpectedSize);

   bool trackEdges = true;
   bool bRobust = true;
   SetMapPointVerticesEdges(  lLocalMapPoints, trackEdges, bRobust );

    //run optimization
    if(pbStopFlag)
        if(*pbStopFlag) {
            std::cout << "in LocalBA: aborting" <<std::endl;
            return;
        }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag) {
            std::cout << "in LocalBA: aborting" <<std::endl;
            bDoMore = false;
        }

    if(bDoMore)
    {
        // Check outlier observations
        std::vector<OutlierMono> vpOutliersMono = FindOutliersMono(chi2_thresh_mono);
        std::vector<OutlierStereo> vpOutliersStereo = FindOutliersStereo(chi2_thresh_stereo);

        for(std::vector<OutlierMono>::iterator vit = vpOutliersMono.begin(); vit !=vpOutliersMono.end(); ++vit){
             g2o::EdgeSE3ProjectXYZ* e = (*vit).e;
             e->setLevel(1); //remove outliers from optimization
        }

        for(std::vector<OutlierStereo>::iterator vit = vpOutliersStereo.begin(); vit !=vpOutliersStereo.end(); ++vit){
             g2o::EdgeStereoSE3ProjectXYZ* e = (*vit).e;
             e->setLevel(1); //remove outliers from optimization
        }

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];
            if(pMP->isBad())
                continue;
            e->setRobustKernel(0);
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    //find outliers after 2nd round of optimization and erase those observations
    std::vector<OutlierMono> vpOutliersMono = FindOutliersMono(chi2_thresh_mono);
    std::vector<OutlierStereo> vpOutliersStereo = FindOutliersStereo(chi2_thresh_stereo);

    for(std::vector<OutlierMono>::iterator vit = vpOutliersMono.begin(); vit !=vpOutliersMono.end(); ++vit){
         OutlierMono outlier = *vit;
         KeyFrame* pKFi = outlier.pKF;
         MapPoint* pMPi = outlier.pMP;
         
        pMap->eraseAssociation(pKFi, pMPi);

    }

    for(std::vector<OutlierStereo>::iterator vit = vpOutliersStereo.begin(); vit !=vpOutliersStereo.end(); ++vit){
         OutlierStereo outlier = *vit;
         KeyFrame* pKFi = outlier.pKF;
         MapPoint* pMPi = outlier.pMP;

         pMap->eraseAssociation(pKFi, pMPi);

    }

    // Recover optimized data
    //Keyframes
    for(std::list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(vertex_map[vertex_name] ));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKFi->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(std::list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        std::string vertex_name = "VertexSBAPointXYZ" + std::to_string(pMP->mnId);
        if(sExcludedMPs.find( vertex_name ) != sExcludedMPs.end()) //test if mappoint is in the set that was excluded from optimization - if so, skip it
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(vertex_map[vertex_name] ) );
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMap->update(pMP);
    }

    ClearLBAFlags(lLocalKeyFrames, lFixedKeyFrames, lLocalMapPoints);

}


std::list<KeyFrame*> LocalBundleAdjustment::FindLocalKFs(KeyFrame *pCentralKF){
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<KeyFrame*> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pCentralKF);
    pCentralKF->mnBALocalForKF = pCentralKF->mnId;

    //const vector<KeyFrame*> vNeighKFs = pCentralKF->GetVectorCovisibleKeyFrames();
    const std::vector<KeyFrame*> vNeighKFs = pMap->getVectorCovisibleKeyFrames(pCentralKF);
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pCentralKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    return lLocalKeyFrames;

}

std::list<MapPoint*> LocalBundleAdjustment::FindLocalMapPoints(const std::list<KeyFrame*> &lLocalKeyFrames, KeyFrame *pCentralKF){

    // Local MapPoints seen in Local KeyFrames
    int i = 0;
    std::list<MapPoint*> lLocalMapPoints;
    for(std::list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        std::vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(std::vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pCentralKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pCentralKF->mnId;
                        i++;
                    }
        }
    }

  return lLocalMapPoints;

}

std::list<KeyFrame*> LocalBundleAdjustment::FindFixedKFs( const std::list<MapPoint*> lLocalMapPoints, KeyFrame *pCentralKF){

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    std::list<KeyFrame*> lFixedCameras;
    for(std::list<MapPoint*>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(std::map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCentralKF->mnId && pKFi->mnBAFixedForKF!=pCentralKF->mnId)
            {
                pKFi->mnBAFixedForKF=pCentralKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

  return lFixedCameras;

}

std::list<Tse3Parent> LocalBundleAdjustment::FindSubmapTiepoints(const std::list<KeyFrame *> &lLocalKeyFrames) {
   std::list<Tse3Parent> submap_tiepoints;
   for(auto it = lLocalKeyFrames.begin(); it != lLocalKeyFrames.end(); ++it ){
       KeyFrame* pKF = *it;
       Tse3Parent tiepoint_data;
       if(pMap->isLocalOrigin(pKF, tiepoint_data)){
            submap_tiepoints.push_back(tiepoint_data);
       }
   }
   return submap_tiepoints;
}

std::list<KeyFrame *> LocalBundleAdjustment::FindAdditionalParentSubmapKFs(const std::list<Tse3Parent> &submap_tiepoints,
                                                                     const std::set<KeyFrame *> &currentKFs) {
    std::list<KeyFrame *> AddnKFs;
    for(auto it  = submap_tiepoints.begin(); it != submap_tiepoints.end(); ++it){
        KeyFrame* KFparent = (*it).pKFref_parent;
        if(!currentKFs.count(KFparent)){
            AddnKFs.push_back(KFparent);
        }

    }
    return AddnKFs;
}


void LocalBundleAdjustment::ClearLBAFlags(const std::list<KeyFrame*> &lLocalKeyFrames, const std::list<KeyFrame*> &lFixedKeyFrames, const std::list<MapPoint*> lLocalMapPoints){
//clear mnBALocalForKF and mnBAFixedForKF flags - these should be eliminated, can just track this locally, no need to append info to keyframes and mappoints cluttering their implementations
   for(std::list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
         KeyFrame* pKFi = *lit;
         pKFi->mnBALocalForKF = std::numeric_limits<unsigned long>::max();
    }

   for(std::list<KeyFrame*>::const_iterator lit=lFixedKeyFrames.begin() , lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
         KeyFrame* pKFi = *lit;
         pKFi->mnBAFixedForKF = std::numeric_limits<unsigned long>::max();
    }


   for(std::list<MapPoint*>::const_iterator lit=lLocalMapPoints.begin() , lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
         MapPoint* pMPi = *lit;
         pMPi->mnBALocalForKF = std::numeric_limits<unsigned long>::max();
    }

}



} // namespace HYSLAM
