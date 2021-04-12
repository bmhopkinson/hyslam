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


Segment::Segment(){}

Segment::Segment(const Segment &seg){
  key_frames = seg.key_frames;
  map_points = seg.map_points;
  Tsim = seg.Tsim.clone();
  Rsim = seg.Rsim.clone(); // deep copy all cv::Mat
  Talign = seg.Talign.clone();
}

Segment& Segment::operator=(const Segment &seg){
  if(this == &seg)  //check for self assignment
    return *this;

  key_frames = seg.key_frames;
  map_points = seg.map_points;
  Tsim = seg.Tsim.clone();
  Rsim = seg.Rsim.clone(); // deep copy all cv::Mat
  Talign = seg.Talign.clone();

  return *this;

}

ImagingBundleAdjustment::ImagingBundleAdjustment(Map* pMap_,  Trajectory* img_trajectory_, g2o::Trajectory &slam_trajectory_, optInfo optParams_)
    : img_trajectory(img_trajectory_),  BundleAdjustment( pMap_, slam_trajectory_, optParams_){

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
  typedef g2o::LinearSolverEigen<MyBlockSolver::PoseMatrixType> MyLinearSolver;

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
  g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  optimizer.setAlgorithm(solver);
  setOptimizer(&optimizer);  //pass to base class BundleAdjustment

}

void ImagingBundleAdjustment::Run(){
  //find continuous segements of tracked Images
  FindTrackedSegments();

  //determine horn transforms between  continous tracked segments and slam cam positions
  DetermineSimilarityTransforms();

  // apply horn transform to mappoints (careful not to apply more than once) and poses
  ApplySimilarityTransforms();
  std::cout << "applied similarity transforms" << std::endl;
  std::this_thread::sleep_for( std::chrono::seconds(5) );

  RotatePosestoAlign();
  std::cout << "rotated poses" << std::endl;
  std::this_thread::sleep_for( std::chrono::seconds(5) );
 // FindAdditionalMapPointMatches(); // THIS ISN'T WORKING RIGHT NOW !!!!!!!!!!!!!!!!!!!!!!!

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


void ImagingBundleAdjustment::FindTrackedSegments(){
  //std::vector<TrajectoryElement> traj_elements = img_trajectory.getTrajectoryElements();

  //for(std::vector<TrajectoryElement>::iterator vit = traj_elements.begin(); vit != traj_elements.end(); ++vit){
  for(auto vit = img_trajectory->begin(); vit != img_trajectory->end(); ++vit){
    std::set<KeyFrame*>  segment_kfs;  //in full trajectory KFs are repeated many times as references for individual frames
    TrajectoryElement te_cur = *vit;
    while(te_cur.tracking_good){
        segment_kfs.insert(te_cur.pRefKF);

        if(vit + 1 == img_trajectory->end()){
          break;
        } else {
          ++vit;
          te_cur = *vit;
          //std::cout << "te_cur.time_stamp: " <<te_cur.time_stamp << endl;
        }

    }

    if(!segment_kfs.empty()){
      Segment tracked_segment;
      tracked_segment.key_frames = segment_kfs;
      segments.push_back(tracked_segment);
      //std::cout << "new segment " << endl;
    }
  }//end for loop

  AssignStrandedKeyFrames();

}

void ImagingBundleAdjustment::AssignStrandedKeyFrames(){
    std::map<KeyFrame*, int> KF_to_segment;
    std::set<KeyFrame*> KFs_in_segments;
    int i = 0;
    for( TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit, ++i) {
        Segment *seg = &(*vit);
        std::set<KeyFrame *> seg_kfs = seg->key_frames;
        for(auto kfit = seg_kfs.begin(); kfit != seg_kfs.end(); ++kfit){
            KF_to_segment[*kfit] = i;  //it's conceivable that KFs are in multiple segments and so only last will be recorded here, but that should be ok
            KFs_in_segments.insert(*kfit);
        }
    }

    std::vector<KeyFrame*> KFs_in_map_temp = pMap->GetAllKeyFrames();
    std::set<KeyFrame*> KFs_in_map(KFs_in_map_temp.begin(), KFs_in_map_temp.end());

    std::set<KeyFrame*> KFs_stranded;
    std::set_difference(KFs_in_map.begin(), KFs_in_map.end(), KFs_in_segments.begin(), KFs_in_segments.end(),
                        std::inserter(KFs_stranded, KFs_stranded.end())  );

    //try to place stranded frames into segments
    //first use parent in spanning tree, if not try children
    for(auto it = KFs_stranded.begin(); it != KFs_stranded.end(); ++it) {
        KeyFrame *pKF = *it;
        KeyFrame *KFparent = pKF->GetParent();
        if (KFparent) {
            auto mit = KF_to_segment.find(KFparent);
            if (mit != KF_to_segment.end()) {
                int idx_seg = mit->second;
                segments[idx_seg].key_frames.insert(pKF);
               // std::cout << "based on parent, found a segment for KF" << pKF->mnId << " in segment: " << idx_seg
                 //         << ", based on parent KF: " << KFparent->mnId << std::endl;
                it = KFs_stranded.erase(it);
            }
        }
    }

    for(auto it = KFs_stranded.begin(); it != KFs_stranded.end(); ++it){
        KeyFrame* pKF = *it;
        std::set<KeyFrame*> KFchildren = pMap->getKeyFrameDB()->getChildren(pKF);
        for(auto cit = KFchildren.begin(); cit != KFchildren.end(); ++cit){
            KeyFrame* KFchild = *cit;
            auto mit = KF_to_segment.find(KFchild);
            if(mit != KF_to_segment.end()){
                int idx_seg = mit->second;
                segments[idx_seg].key_frames.insert(pKF);
               // std::cout << "based on child, found a segment for KF"  << pKF->mnId << " in segment: "  << idx_seg << ", based on child KF: " << KFchild->mnId << std::endl;
                it = KFs_stranded.erase(it);
                break;
            }
        }
    }

   // std::cout << "unable to assign: " << KFs_stranded.size() << " stranded KeyFrames" << std::endl;
    //set unassigned keyframes "bad" e.g. erase them
    for(auto it = KFs_stranded.begin(); it != KFs_stranded.end(); ++it) {
        KeyFrame *pKF = *it;
      //  std::cout << "about to set KF bad: " << pKF->mnId << std::endl;
        pMap->SetBadKeyFrame(pKF);
    }
// std::cout << "finished setting stranded KeyFrames as bad   " << std::endl;

}

void ImagingBundleAdjustment::DetermineSimilarityTransforms(){

  for(TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit){
    Segment* seg = &(*vit);
    std::set<KeyFrame*> seg_kfs = seg->key_frames;

    cv::Mat P1; //new coords
    cv::Mat P2; //old coords
    int nKF = 0;
    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;

//      Isometry3 p_slam =  trajectory.poseAtTime(pKFi->mTimeStamp); //in camera to world convention
//      Eigen::Matrix<double, 3, 1> t_slam = p_slam.translation();
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
          seg->Tsim = Thorn.clone();
          seg->Rsim = Rhorn.clone();
        //transforms.push_back( std::make_pair(Thorn.clone(), Rhorn.clone()) );
        } else {
          cv::Mat emptyMat;
          //transforms.push_back( std::make_pair(emptyMat.clone(), emptyMat.clone()) );
          seg->Tsim = emptyMat.clone();
          seg->Rsim = emptyMat.clone();
        }

    } else {
      cv::Mat emptyMat;
      //transforms.push_back( std::make_pair(emptyMat.clone(), emptyMat.clone()) );
      seg->Tsim = emptyMat.clone();
      seg->Rsim = emptyMat.clone();

    }

  } //end for loop

}

void ImagingBundleAdjustment::ApplySimilarityTransforms(){ //KFs and mpts to which similarity is succesfully applied will be added to optimization lists
  for(TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit){
    Segment* seg = &(*vit);
    std::set<KeyFrame*> seg_kfs = seg->key_frames;
    cv::Mat Thorn =  seg->Tsim;
    cv::Mat Rhorn =  seg->Rsim;

    if(Thorn.empty()){
      continue;
    }

    std::set<MapPoint*> segment_mappts_all;
    std::set<MapPoint*> segment_mappts_valid;
    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;
      if(pKFi->Thorn_applied)
      { continue; }

      pKFi->storePose();

      //apply similarity transform to poses - gets a little complicated b/c rotatation must remain a pure rotation
      cv::Mat R_old = pKFi->GetRotation();
      cv::Mat R_new = R_old*Rhorn.inv();
      cv::Mat Ow_old = pKFi->GetCameraCenter();
      Ow_old.push_back(cv::Mat::ones(1,1,CV_32F)); //make homogeneous
      cv::Mat Ow_new = Thorn * Ow_old;
      cv::Mat t_cw_new = R_new * (-1*Ow_new.rowRange(0,3));
      cv::Mat Tcw_new = cv::Mat::eye(4,4,CV_32F);
      t_cw_new.copyTo( Tcw_new.rowRange(0,3).col(3) );
      R_new.copyTo( Tcw_new.rowRange(0,3).colRange(0,3) );

      pKFi->SetPose(Tcw_new);
      pKFi->Thorn_applied = true;
      KFs_to_optimize.push_back(pKFi);

      //now collect mappoints
      std::set<MapPoint*> vpMPs = pKFi->GetMapPoints();
      for(std::set<MapPoint*>::iterator mpit = vpMPs.begin(); mpit != vpMPs.end(); ++mpit){
        segment_mappts_all.insert(*mpit);
    //    std::cout << "(*mpit)->mnId: " << (*mpit)->mnId << std::endl;
      }

    } //end for loop on keyframes in segment

    //tranfrom  map points - make sure transformation is only applied once per mappoint, but that all relevant map points are transfered
    for(std::set<MapPoint*>::iterator mpit = segment_mappts_all.begin(); mpit != segment_mappts_all.end(); ++mpit){
      MapPoint* mpt = *mpit;
//      std::cout << "mpt->mnId: " << mpt->mnId << std::endl;
      if(!mpt->Thorn_applied){
        mpt->applyTransform(Thorn);
        mpts_to_optimize.push_back(mpt);
        segment_mappts_valid.insert(mpt);

      }
    }
    seg->map_points = segment_mappts_valid;

  } //end for loop on segments

}

void ImagingBundleAdjustment::RotatePosestoAlign(){
  //first clear all Thorn_applied flags for key_frames, mappts have been exclusively assigned to segments so shouldn't be an issue
  for(TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit){
    Segment* seg = &(*vit);
    std::set<KeyFrame*> seg_kfs = seg->key_frames;
    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;
      pKFi->Thorn_applied = false;
    }
  }

  //calculate rotations
  int  i = 0;
  for(TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit){
    Segment* seg = &(*vit);
    std::set<KeyFrame*> seg_kfs = seg->key_frames;
    std::vector<cv::Mat> Tslam;
    std::vector<cv::Mat> Tseg;
    std::string Tslam_fname = "./data/Tslam_data_"+std::to_string(i) + ".txt";
    std::string Tseg_fname  = "./data/Tseg_data_" +std::to_string(i) + ".txt";
    std::ofstream Tslam_file;
    std::ofstream Tseg_file;
    Tslam_file.open(Tslam_fname.c_str());
    Tseg_file.open(Tseg_fname.c_str());
    int j = 0;

    if(seg->Tsim.empty()){ //couldnt' determine valid transform previously so skip here again
      continue;
    }

    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;
      cv::Mat Tslam_i =  Converter::Iso3tocvMat( trajectory.poseAtTime(pKFi->mTimeStamp) ); //in camera to world convetion
      Tslam_i = Tslam_i * pKFi->camera.Tcam; //transform from slam cam position to this cam's estimate position
      Tslam_i = Tslam_i.inv();
      Tslam.push_back( Tslam_i.clone() );
      Tseg.push_back( pKFi->GetPose() );  //in world to camera convention

      Tslam_file << j << "\n" << Tslam_i << std::endl;
      Tseg_file << j << "\n" << pKFi->GetPose() << std::endl;
      j++;

    }
    Tslam_file.close();
    Tseg_file.close();
    i++;

    cv::Mat Talign = PoseAlignmentTransform(Tslam, Tseg);
    seg->Talign = Talign.clone();
    cv::Mat Talign_inv = Talign.inv();

    //apply rotation
    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;
      if(pKFi->Thorn_applied)
      { continue; }

      cv::Mat Tcw_old = pKFi->GetPose();
      cv::Mat Tcw_new = Tcw_old*Talign_inv;

//      cv::Mat Tcw_cur = pKFi->GetPose();
//      cv::Mat Tcw_new = Tcw_cur;
//      R_new.copyTo( Tcw_new.rowRange(0,3).colRange(0,3) );

      pKFi->SetPose(Tcw_new);
      pKFi->Thorn_applied = true;

    }

    std::set<MapPoint*> map_points = seg->map_points;
    for(std::set<MapPoint*>::iterator sit = map_points.begin(); sit != map_points.end(); ++sit){
      MapPoint* mpt = *sit;
      mpt->applyTransform(Talign);
    //  mpt->UpdateNormalAndDepth();
      pMap->getMapPointDB()->updateEntry(mpt);
    }

  }

}

void ImagingBundleAdjustment::FindAdditionalMapPointMatches(){
//attempt to fuse any visible untracked mappoints into keyframes - especially hoping to get inter-segment viewing of mappoints
  std::cout << "FindAdditionalMapPointMatches" << std::endl;
  for(TrackedSegments::iterator vit= segments.begin(); vit != segments.end(); ++vit){

    Segment* seg = &(*vit);
    std::set<KeyFrame*> seg_kfs = seg->key_frames;
    for(std::set<KeyFrame*>::iterator sit = seg_kfs.begin(); sit != seg_kfs.end(); ++sit){
      KeyFrame* pKFi = *sit;
      std::vector<MapPoint*> visible_mpts, fuse_candidates;
      pMap->visibleMapPoints(pKFi, visible_mpts);

      for(std::vector<MapPoint*>::iterator mpit = visible_mpts.begin(); mpit != visible_mpts.end(); ++mpit){
        MapPoint* mpt = *mpit;
        if(!pKFi->isMapPointMatched(mpt)){
          fuse_candidates.push_back(mpt);
        }
      }

      std::cout << "found visible mappts for KF: " << pKFi->mnId << std::endl;
      //determine fuse candidates by excluding any mappoints already tracked in keyframe
      FeatureMatcher matcher;
      float search_radius_thresh = 5.0; //larger than default of 3.00
      float reprojection_err_thresh = 10.0; //allow a bit more than default of 5.99
      std::map<std::size_t, MapPoint*> fuse_matches;
      matcher.Fuse(pKFi,fuse_candidates, fuse_matches, search_radius_thresh, reprojection_err_thresh);

      //merge fuse matches
      for(auto it = fuse_matches.begin(); it != fuse_matches.end(); ++it){
        size_t idx = it->first;
        MapPoint* lm_fuse = it->second;
        MapPoint* lm_current = pKFi->hasAssociation(idx);
        if(lm_current)
        {
            if(!lm_current->isBad())
            {
                std::cout << "Fusing mappoint" << std::endl;
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


	 std::cout << "fused visible mappts for KF: " << pKFi->mnId << std::endl;
      // Update points - this is done in LocalMapping after fusing - seems like it should be embeded in the fuse routine
      std::vector<MapPoint*> vpMapPointMatches = pKFi->GetMapPointMatches();
      for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
      {
          MapPoint* pMP=vpMapPointMatches[i];
          if(pMP)
          {
              if(!pMP->isBad())
              {
                  pMap->getMapPointDB()->updateEntry(pMP);

              }
          }
      }

      // Update connections in covisibility graph- this is done in LocalMapping after fusing - seems like it should be embeded in the fuse routine
      std::cout << "about to update connections for KF: " << pKFi->mnId <<std::endl;
     // pKFi->UpdateConnections();


    }
  }

}

}
