#include "ImagingBundleAdjustment.h"
#include "OptHelpers.h"
#include <FeatureMatcher.h>
#include <MapPointDB.h>
#include <Converter.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <limits>
#include <thread>
#include <algorithm>

namespace HYSLAM
{
    

ImagingBundleAdjustment::ImagingBundleAdjustment(Map* pMap_,  Trajectory* img_trajectory_, g2o::Trajectory &slam_trajectory_, FeatureFactory* factory, optInfo optParams_)
    : img_trajectory(img_trajectory_), feature_factory(factory),  BundleAdjustment( pMap_, slam_trajectory_, optParams_){

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
  typedef g2o::LinearSolverEigen<MyBlockSolver::PoseMatrixType> MyLinearSolver;

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
  g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  optimizer.setAlgorithm(solver);
  setOptimizer(&optimizer);  //pass to base class BundleAdjustment

}

void ImagingBundleAdjustment::Run(){

  //determine horn transforms between  submaps using  slam cam positions
  DetermineSimilarityTransforms();

  // apply horn transform to keyframes  and mappoints (careful not to apply more than once)
  ApplySimilarityTransforms();
  std::cout << "applied similarity transforms" << std::endl;
  std::this_thread::sleep_for( std::chrono::seconds(5) );

  RotatePosestoAlign();
  std::cout << "rotated poses" << std::endl;

//all submaps have been transformed into a common reference frame and "registered" so make this official:
  std::vector<std::shared_ptr<Map>> submaps = pMap->getSubmaps();
  for(auto it = submaps.begin(); it != submaps.end(); ++it) {
    (*it)->registerWithParent();
  }

  std::this_thread::sleep_for( std::chrono::seconds(5) );
 // FindAdditionalMapPointMatches();  //this really needs to be BVH accelerated and should be done after an initial g2o optimization round

  //eventually: handle untracked frames

  //set up optimization and run
  SetKeyFrameVertices(KFs_to_optimize, false);
  std::cout << "set keyframe vertices" << std::endl;
  SetImagingVertices(KFs_to_optimize);
    std::cout << "set imaging vertices" << std::endl;
  SetImagingEdges(KFs_to_optimize);
    std::cout << "set imaging edges" << std::endl;
  //find total mappoint observations for preallocation of edges vector // should put this in a funciton
  int totalMPobs = 0;
  for(std::list<KeyFrame*>::iterator kfit = KFs_to_optimize.begin(); kfit != KFs_to_optimize.end(); ++kfit){
    KeyFrame* pKFi = *kfit;
    int minObs = 1;
    int MPobs = pKFi->TrackedMapPoints(minObs);
    totalMPobs += MPobs;
  }


  const int nExpectedSize = static_cast<int>(1.2*totalMPobs); //add a small buffer to total number of expected observations to ensure sufficient room is saved
  vpEdgesMono.reserve(nExpectedSize);
  vpEdgeKFMono.reserve(nExpectedSize);
  vpMapPointEdgeMono.reserve(nExpectedSize);
  vpEdgesStereo.reserve(nExpectedSize);
  vpEdgeKFStereo.reserve(nExpectedSize);
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  bool trackEdges = true;
  bool bRobust = true;
  SetMapPointVerticesEdges(  mpts_to_optimize, trackEdges, bRobust );
    std::cout << "set mappoint edges/vertices" << std::endl;
  optimizer.initializeOptimization();
  optimizer.optimize(5);
    std::cout << "finished first optimization" << std::endl;
  // Check outlier observations and remove from BA

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
  //optimize again only with inliers
  int optLevel = 0; //only optimize with inliers (level 0)
  optimizer.initializeOptimization(optLevel);
  optimizer.optimize(10);   
   std::cout << "finished second optimization" << std::endl;

  // Recover optimized data
  //Keyframes
  for(std::list<KeyFrame*>::iterator lit=KFs_to_optimize.begin(), lend=KFs_to_optimize.end(); lit!=lend; lit++)
  {
    KeyFrame* pKFi = *lit;
    std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
    g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(vertex_map[vertex_name] ));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    pKFi->SetPose(Converter::toCvMat(SE3quat));
   // std::cout << "set keyframe" << std::endl;
  }

  //Tcam - optimized SLAM camera to imaging camera transform
  std::string Tcam_vertex_name = "VertexSE3_Tcam";
  g2o::VertexSE3* Tcam_vert = static_cast<g2o::VertexSE3*>(optimizer.vertex(vertex_map[Tcam_vertex_name] ));
  cv::Mat Tcam_opt = Converter::Iso3tocvMat(Tcam_vert->estimate());
  Tcam_opt = Tcam_opt.inv();
  for(std::list<KeyFrame*>::iterator lit=KFs_to_optimize.begin(), lend=KFs_to_optimize.end(); lit!=lend; lit++)
  {
    KeyFrame* pKFi = *lit;
    pKFi->camera.Tcam_opt = Tcam_opt.clone();
  }

  //Points
  for(std::list<MapPoint*>::iterator lit=mpts_to_optimize.begin(), lend=mpts_to_optimize.end(); lit!=lend; lit++)
  {
    MapPoint* pMP = *lit;
    std::string vertex_name = "VertexSBAPointXYZ" + std::to_string(pMP->mnId);
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(vertex_map[vertex_name] ) );
       //  std::cout << "retrieved vPoint" << std::endl;
    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
     //     std::cout << "set world position" << std::endl;
    //    pMP->UpdateNormalAndDepth();
  }



}


void ImagingBundleAdjustment::DetermineSimilarityTransforms(){
    std::cout << "DetermineSimilarityTransforms()"  << std::endl;
    std::vector<std::shared_ptr<Map>> submaps = pMap->getSubmaps();
    for(auto it = submaps.begin(); it != submaps.end(); ++it){
        std::shared_ptr<Map> submap = *it;
        std::vector<KeyFrame*> kfs_submap = submap->GetAllKeyFrames();

        cv::Mat P1; //new coords
        cv::Mat P2; //old coords
        int nKF = 0;
        for(auto it2 = kfs_submap.begin(); it2 != kfs_submap.end(); ++it2){
            KeyFrame* pKFi = *it2;

            cv::Mat Tslam_i =  Converter::Iso3tocvMat( trajectory.poseAtTime(pKFi->mTimeStamp) ); //in camera to world convention
            Tslam_i = Tslam_i * pKFi->camera.Tcam; //transform from slam cam position to this cam's estimate position
            cv::Mat t_slam_cv(1,3, CV_32F);
            t_slam_cv.at<float>(0,0) = Tslam_i.at<float>(0,3);
            t_slam_cv.at<float>(0,1) = Tslam_i.at<float>(1,3);
            t_slam_cv.at<float>(0,2) = Tslam_i.at<float>(2,3);
            P1.push_back(t_slam_cv.clone());
            cv::Mat p_img = pKFi->GetCameraCenter().t();
            P2.push_back(p_img);

            nKF++;
        }

        if(nKF>4){
            cv::Mat P1_t = P1.t();
            cv::Mat P2_t = P2.t();
            bool FixScale = 0; //fixed scale = 1; variable scale = 0
            cv::Mat Rhorn; //rotation matrix computed by ComputeSim3_Horn
            cv::Mat Thorn = ComputeSim3_Horn( P1_t, P2_t, FixScale, Rhorn);

            bool validSim3 = 1;
            for(int i = 0; i < Thorn.cols; i++){    //ensure sim3 is valid - with few data points it may not be//this test should be done in ComputeSim3_Horn function
                for(int j = 0; j < Thorn.rows; j++){
                    if(isnan(Thorn.at<float>(i,j))){
                        validSim3 = 0;
                    }
                }
            }

            if(validSim3){
                SubmapData submap_datum;
                submap_datum.Tsim = Thorn.clone();
                submap_datum.Rsim = Rhorn.clone();
                submap_data[submap] = submap_datum;
                //transforms.push_back( std::make_pair(Thorn.clone(), Rhorn.clone()) );
            } else {
                SubmapData submap_datum;
                cv::Mat emptyMat;
                submap_datum.Tsim = emptyMat.clone();
                submap_datum.Rsim = emptyMat.clone();
                submap_data[submap] = submap_datum;
            }

        } else {
            SubmapData submap_datum;
            cv::Mat emptyMat;
            submap_datum.Tsim = emptyMat.clone();
            submap_datum.Rsim = emptyMat.clone();
            submap_data[submap] = submap_datum;
        }

  } //end for loop

}

void ImagingBundleAdjustment::ApplySimilarityTransforms(){ //KFs and mpts to which similarity is succesfully applied will be added to optimization lists
    std::cout << "ApplySimilarityTransforms()"  <<std::endl;
    std::vector<std::shared_ptr<Map>> submaps = pMap->getSubmaps();
    for(auto it = submaps.begin(); it != submaps.end(); ++it) {
        std::shared_ptr<Map> submap = *it;
        std::vector<KeyFrame *> kfs_submap = submap->GetAllKeyFrames();
        std::vector<MapPoint *> mpts_submap = submap->GetAllMapPoints();

        SubmapData submap_datum = submap_data[submap];
        cv::Mat Thorn = submap_datum.Tsim;
        cv::Mat Rhorn = submap_datum.Rsim;

        if (Thorn.empty()) {
            continue;
        }

        for (std::vector<KeyFrame *>::iterator sit = kfs_submap.begin(); sit != kfs_submap.end(); ++sit) {
            KeyFrame *pKFi = *sit;
            if (pKFi->Thorn_applied) { continue; }

            pKFi->storePose();

            //apply similarity transform to poses - gets a little complicated b/c rotatation must remain a pure rotation
            cv::Mat R_old = pKFi->GetRotation();
            cv::Mat R_new = R_old * Rhorn.inv();
            cv::Mat Ow_old = pKFi->GetCameraCenter();
            Ow_old.push_back(cv::Mat::ones(1, 1, CV_32F)); //make homogeneous
            cv::Mat Ow_new = Thorn * Ow_old;
            cv::Mat t_cw_new = R_new * (-1 * Ow_new.rowRange(0, 3));
            cv::Mat Tcw_new = cv::Mat::eye(4, 4, CV_32F);
            t_cw_new.copyTo(Tcw_new.rowRange(0, 3).col(3));
            R_new.copyTo(Tcw_new.rowRange(0, 3).colRange(0, 3));

            pKFi->SetPose(Tcw_new);
            pKFi->Thorn_applied = true;
            KFs_to_optimize.push_back(pKFi);

        } //end for loop on keyframes in segment


        for (auto mpit = mpts_submap.begin(); mpit != mpts_submap.end(); ++mpit) {
            MapPoint *mpt = *mpit;
//      std::cout << "mpt->mnId: " << mpt->mnId << std::endl;
            if (!mpt->Thorn_applied) {
                mpt->applyTransform(Thorn);
                mpt->flagTransformApplied();
                mpts_to_optimize.push_back(mpt);
            }
        }
    }//end loop on submaps

}

void ImagingBundleAdjustment::RotatePosestoAlign(){
    std::cout << "RotatePosesToAlign()"  << std::endl;
  //first clear all Thorn_applied flags for key_frames, mappts have been exclusively assigned to segments so shouldn't be an issue
    for(auto it = KFs_to_optimize.begin(); it != KFs_to_optimize.end(); ++it){
        (*it)->Thorn_applied = false;
    }

    //calculate rotations
    int  i = 0;
    std::vector<std::shared_ptr<Map>> submaps = pMap->getSubmaps();
    for(auto it = submaps.begin(); it != submaps.end(); ++it) {
        std::shared_ptr<Map> submap = *it;
        std::vector<KeyFrame *> kfs_submap = submap->GetAllKeyFrames();
        std::vector<MapPoint *> mpts_submap = submap->GetAllMapPoints();
        SubmapData submap_datum = submap_data[submap];

        std::vector<cv::Mat> Tslam;
        std::vector<cv::Mat> Tseg;
        std::string Tslam_fname =  "./data/Tslam_data_" + std::to_string(i) + ".txt";
        std::string Timgslam_fname = "./data/Timgslam_data_" + std::to_string(i) + ".txt";
        std::string Tseg_fname  = "./data/Tseg_data_" +std::to_string(i) + ".txt";
        std::ofstream Tslam_file;
        std::ofstream Timgslam_file;
        std::ofstream Tseg_file;
        Tslam_file.open(Tslam_fname.c_str());
        Timgslam_file.open(Timgslam_fname.c_str());
        Tseg_file.open(Tseg_fname.c_str());

        Timgslam_file << "SlamToImg Transform: " << "\n" << (*kfs_submap.begin())->camera.Tcam << std::endl;
        Tseg_file  << "SlamToImg Transform: " << "\n" << (*kfs_submap.begin())->camera.Tcam << std::endl;

        int j = 0;

        if(submap_datum.Tsim.empty()){ //couldnt' determine valid transform previously so skip here again
            continue;
        }

        for(auto sit = kfs_submap.begin(); sit != kfs_submap.end(); ++sit){
            KeyFrame* pKFi = *sit;
            cv::Mat Tslam_i =  Converter::Iso3tocvMat( trajectory.poseAtTime(pKFi->mTimeStamp) ); //in camera to world convetion
            Tslam_file << j << "\n" << Tslam_i << std::endl;

            Tslam_i = Tslam_i * pKFi->camera.Tcam; //transform from slam cam position to this cam's estimate position
            Tslam_i = Tslam_i.inv();
            Tslam.push_back( Tslam_i.clone() );
            Tseg.push_back( pKFi->GetPose() );  //in  world to camera convention

            Timgslam_file << j << "\n" << Tslam_i.inv() << std::endl;
            Tseg_file << j << "\n" << pKFi->GetPoseInverse() << std::endl;
            j++;

        }
        Tslam_file.close();
        Timgslam_file.close();
        Tseg_file.close();
        i++;

        cv::Mat Talign = PoseAlignmentTransform(Tslam, Tseg);
        submap_datum.Talign = Talign.clone();
        cv::Mat Talign_inv = Talign.inv();
        submap_data[submap] = submap_datum;


        //apply rotation
        for(auto sit = kfs_submap.begin(); sit != kfs_submap.end(); ++sit){
            KeyFrame* pKFi = *sit;
            if(pKFi->Thorn_applied)
            { continue; }

            cv::Mat Tcw_old = pKFi->GetPose();
            cv::Mat Tcw_new = Tcw_old*Talign_inv;

            pKFi->SetPose(Tcw_new);
            pKFi->Thorn_applied = true;

        }

        for(auto sit = mpts_submap.begin(); sit != mpts_submap.end(); ++sit){
            MapPoint* mpt = *sit;
            mpt->applyTransform(Talign);
            pMap->update(mpt);
        }

    }
}

void ImagingBundleAdjustment::FindAdditionalMapPointMatches(){
//attempt to fuse any visible untracked mappoints into keyframes - especially hoping to get inter-segment viewing of mappoints
    std::cout << "FindAdditionalMapPointMatches" << std::endl;

    std::vector<KeyFrame*> all_kfs = pMap->GetAllKeyFrames();
    for(auto it = all_kfs.begin(); it != all_kfs.end(); ++it){
      KeyFrame* pKFi = *it;
      std::vector<MapPoint*> visible_mpts, fuse_candidates;
      pMap->visibleMapPoints(pKFi, visible_mpts);

      for(std::vector<MapPoint*>::iterator mpit = visible_mpts.begin(); mpit != visible_mpts.end(); ++mpit){
        MapPoint* mpt = *mpit;
        if(!pKFi->isMapPointMatched(mpt)){
          fuse_candidates.push_back(mpt);
        }
      }

    //  std::cout << "found fuse candidate mappts for KF: " << pKFi->mnId << ", N candidates: " << fuse_candidates.size() <<std::endl;
      //determine fuse candidates by excluding any mappoints already tracked in keyframe
      std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();
      float search_radius_thresh = 5.0; //larger than default of 3.00
      float reprojection_err_thresh = 10.0; //allow a bit more than default of 5.99
      std::map<std::size_t, MapPoint*> fuse_matches;
      matcher->Fuse(pKFi,fuse_candidates, fuse_matches, search_radius_thresh, reprojection_err_thresh);

      //merge fuse matches
      for(auto it = fuse_matches.begin(); it != fuse_matches.end(); ++it){
        size_t idx = it->first;
        MapPoint* lm_fuse = it->second;
        MapPoint* lm_current = pKFi->hasAssociation(idx);
        if(lm_current)
        {
            if(!lm_current->isBad())
            {
                //std::cout << "Fusing mappoint" << std::endl;
                if(lm_current->Observations() > lm_fuse->Observations()){

                    pMap->replaceMapPoint(lm_fuse, lm_current);
                }
                else{
                    pMap->replaceMapPoint(lm_current, lm_fuse);
                }
            }
        }
        else
        {
            pMap->addAssociation(pKFi, idx, lm_fuse, true);
        }
      }


	// std::cout << "fused visible mappts for KF: " << pKFi->mnId << " , N_mpts fused: " << fuse_matches.size() <<  std::endl;
      // Update points - this is done in LocalMapping after fusing - seems like it should be embeded in the fuse routine
      std::vector<MapPoint*> vpMapPointMatches = pKFi->GetMapPointMatches();
      for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
      {
          MapPoint* pMP=vpMapPointMatches[i];
          if(pMP)
          {
              if(!pMP->isBad())
              {
                  pMap->update(pMP);

              }
          }
      }

      // Update connections in covisibility graph- this is done in LocalMapping after fusing - seems like it should be embeded in the fuse routine
     // std::cout << "about to update connections for KF: " << pKFi->mnId <<std::endl;
      pMap->update(pKFi);


    }
}

}
