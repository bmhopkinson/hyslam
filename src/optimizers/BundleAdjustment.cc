#include "BundleAdjustment.h"
#include "OptHelpers.h"
#include <SensorData.h>
#include <FeatureExtractorSettings.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/sba_accessory_cam.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/slam3d_addons/SE3_sensor_edges.h"
#include "g2o/types/slam3d/vertex_se3.h"


#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>

#include "Converter.h"


#include<vector>
#include<math.h>

namespace HYSLAM
{

BundleAdjustment::BundleAdjustment(Map* pMap_, g2o::Trajectory &trajectory_, optInfo optParams_ ) :
            pMap(pMap_) , trajectory(trajectory_), optParams(optParams_){}
//BundleAdjustment::BundleAdjustment(Map* pMap_, optInfo optParams_ ) :
//            pMap(pMap_) ,optParams(optParams_){}

BundleAdjustment::~BundleAdjustment()
{}

void BundleAdjustment::SetKeyFrameVertices(const std::list<KeyFrame*> &lKeyFrames, bool fixed  ){
   for(std::list<KeyFrame*>::const_iterator lit=lKeyFrames.begin(), lend=lKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(vertex_id);
        std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
        vertex_map.insert(std::make_pair(vertex_name, vertex_id));
        vertex_id++;

        if(fixed){
           vSE3->setFixed(true);
        }
        optimizer->addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

}


void BundleAdjustment::SetIMUEdges( const std::list<KeyFrame*> &lKeyFrames ){

    for(std::list<KeyFrame*>::const_iterator lit=lKeyFrames.begin(), lend=lKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->isBad())
            continue;

        if(pKFi->getSensorData().isImuValid()){  //if valid imu data is available - set constraint
      //      std::cout << "incorporating imu" << "\t";
           // ImuData imu_data = pKFi->getSensorData().getImu();
            Eigen::Quaterniond _q = Converter::toQuatEigen(pKFi->getSensorData().getQuat());
            Eigen::Matrix<double, 4, 1> q;
            q <<  _q.x() , _q.y() , _q.z(), _q.w();   //double check this convention

            g2o::EdgeIMUQuat* eIMU = new g2o::EdgeIMUQuat();
            std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
            eIMU->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_map[vertex_name]) ) );
            eIMU->setMeasurement(q); // set measured quaternion.
            eIMU->setInformation(optParams.Info_IMU*Eigen::Matrix4d::Identity());
            eIMU->setLevel(0);  // 0 = include; 1 = exclude
            optimizer->addEdge(eIMU);
        }
    }

}

void BundleAdjustment::SetDepthEdges( const std::list<KeyFrame*> &lKeyFrames ){

 for(std::list<KeyFrame*>::const_iterator lit=lKeyFrames.begin(), lend=lKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->isBad())
            continue;

        if(pKFi->getSensorData().isDepthValid()){  //if valid depth data is available - set constraint
       //     std::cout << "incorporating depth" << "\t";
            g2o::Vector1d depthData;
            depthData << pKFi->getSensorData().getDepth();

            g2o::EdgeDepth* eDepth = new g2o::EdgeDepth();
            std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
            eDepth->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_map[vertex_name]) ) );
            eDepth->setMeasurement(depthData);
            eDepth->setInformation(optParams.Info_Depth*Eigen::Matrix<double, 1, 1>::Identity());
            eDepth->setLevel(0);
            optimizer->addEdge(eDepth);
        }
    }

}

void BundleAdjustment::SetGPSEdges( const std::list<KeyFrame*> &lKeyFrames ){
    //add GPS constraints
    //first estimate optimal GPS to ORB_SLAM2 coordinate system transform using Horn 1987 algorithm - use all available keyframe data
    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    cv::Mat P1;
    cv::Mat P2;
    int nGPSdata = 0;
    for(std::vector<KeyFrame*>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); vit++){
        KeyFrame* pKFi = *vit;
        if(pKFi->getSensorData().isGPSValid()){
            nGPSdata++;
            cv::Mat p = pKFi->GetCameraCenter().t();
            P1.push_back(p);
            cv::Mat temp(1,3,CV_32F);
            std::vector<double> relpos = pKFi->getSensorData().getGPSRel();
            temp.at<float>(0,0) = relpos[0];
            temp.at<float>(0,1) = relpos[1];
            temp.at<float>(0,2) = relpos[2];
            P2.push_back(temp);
        }
    }

    if(nGPSdata > 4){
      cv::Mat Porb = P1.t();   //transpose
      cv::Mat Pgps = P2.t();

      bool FixScale = 0; //fixed scale = 1; variable scale = 0
      cv::Mat Rhorn; //rotation matrix computed by ComputeSim3_Horn
      cv::Mat Thorn = ComputeSim3_Horn( Porb, Pgps, FixScale, Rhorn);
  //    if(ncalls % 20 == 0){
   //      std::cout << "sim3 transform: " << Thorn << endl;
   //   }

      bool validSim3 = 1;
      for(int i = 0; i < Thorn.cols; i++){    //ensure sim3 is valid - with few gps locations early in SLAM it may not be
          for(int j = 0; j < Thorn.rows; j++){
             if(isnan(Thorn.at<float>(i,j))){
                  validSim3 = 0;
             }
          }
      }

    //create GPS to Pose edges
      if(validSim3){
          for(std::list<KeyFrame*>::const_iterator lit=lKeyFrames.begin(), lend=lKeyFrames.end(); lit!=lend; lit++)
          {
            KeyFrame* pKFi = *lit;
            if(pKFi->isBad())
               continue;

              if(pKFi->getSensorData().isGPSValid()){ //if valid GPS data is available - set constraint
                 //transform gps data
                std::vector<double> relpos = pKFi->getSensorData().getGPSRel();
              //  std::cout << "gps pos: "  << relpos[0] << " " << relpos[1] << std::endl;
                cv::Mat p(4,1,CV_32F);
                p.at<float>(0,0) = relpos[0];
                p.at<float>(1,0) = relpos[1];
                p.at<float>(2,0) = relpos[2];
                p.at<float>(3,0) = 1.0000;
                cv::Mat p_orb = Thorn*p;
                g2o::Vector3d gpsData(Converter::toVector3d(p_orb( cv::Range(0,3),cv::Range::all() ) )); //take first 3 elements of transformed vector (cv range is exclusive like python)
                Eigen::Matrix<double, 3,3> gpsInf  = Rotate_GpsError(pKFi->getSensorData().getGPSerr(), Rhorn);  // rotate gps error into orb coordinate frame

                g2o::EdgeGPS* eGPS = new g2o::EdgeGPS();
                std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
                eGPS->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_map[vertex_name]) ) );
                eGPS->setMeasurement(gpsData);
                eGPS->setInformation(optParams.Info_GPS*gpsInf);
                eGPS->setLevel(0);
                optimizer->addEdge(eGPS);
              }
         }// end loop on local keyframes
      }// end if  on validSim3
    }//end if on nGPSdata > 4

}

void BundleAdjustment::SetMapPointVerticesEdges( const std::list<MapPoint*> lMapPoints, bool trackEdges, bool bRobust ){
    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int i  = 0;
    for(std::list<MapPoint*>::const_iterator lit=lMapPoints.begin(), lend=lMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        //unsigned long id = pMP->mnId+maxKFid+1;
        vPoint->setId(vertex_id);
        std::string vertex_name_mp = "VertexSBAPointXYZ" + std::to_string(pMP->mnId);
        vertex_map.insert(std::make_pair(vertex_name_mp, vertex_id));
        vertex_id++;

        vPoint->setMarginalized(true);
        optimizer->addVertex(vPoint);

        const std::map<KeyFrame*,size_t> observations = pMP->GetObservations();
		int n_mono = 0;
		int n_stereo = 0;
        int nEdges = 0;
        //Set edges
        for(std::map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {

            KeyFrame* pKFi = mit->first;
            const FeatureViews views = pKFi->getViews();
            FeatureExtractorSettings orb_params = views.orbParams();
            const Camera camera = pKFi->getCamera();
            
            std::string vertex_name_kf = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
            bool kf_vertex_missing = (vertex_map.find(vertex_name_kf) == vertex_map.end() );
            if(pKFi->isBad() || kf_vertex_missing  )
                continue;
            nEdges++;

            const cv::KeyPoint kpUn = views.keypt(mit->second);
            if(views.uR(mit->second)<0) //not all stereo keypoints have a stereomatch
            {
				n_mono++;
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_mp]) ) ) ;
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_kf]) ) );

                e->setMeasurement(obs);
                const float invSigma2 = 1/orb_params.determineSigma2(kpUn.size); //mvInvLevelSigma2[kpUn.octave];
                
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust){
                   g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                   e->setRobustKernel(rk);
                   rk->setDelta(thHuberMono);
                }

                e->fx = camera.fx();
                e->fy = camera.fy();
                e->cx = camera.cx();
                e->cy = camera.cy();

                optimizer->addEdge(e);

                if(trackEdges){
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
            }
            else // Stereo observation
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = views.uR(mit->second);
                
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();


                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_mp]) ) ) ;
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_kf]) ) );

                e->setMeasurement(obs);

                const float invSigma2 = 1/orb_params.determineSigma2(kpUn.size); //mvInvLevelSigma2[kpUn.octave];

                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust){
                   g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                   e->setRobustKernel(rk);
                   rk->setDelta(thHuberStereo);
                }

                e->fx = camera.fx();
                e->fy = camera.fy();
                e->cx = camera.cx();
                e->cy = camera.cy();
                e->bf = camera.mbf;

                optimizer->addEdge(e);

                if(trackEdges){
                  vpEdgesStereo.push_back(e);
                  vpEdgeKFStereo.push_back(pKFi);
                  vpMapPointEdgeStereo.push_back(pMP);
               }
               n_stereo++;
               
            }

        }
        
        if(nEdges==0)
        {
            std::cout << "excluding mappoint: "<< vertex_name_mp  << std::endl;
            sExcludedMPs.insert( vertex_name_mp );
            optimizer->removeVertex( vPoint );
        }

        i++;
      //  std::cout << "MP: " << pMP->mnId <<", n_mono: " << n_mono <<" , n_stereo: " << n_stereo << std::endl ;
    } //end loop on mappoints

}



void BundleAdjustment::SetImagingVertices(const std::list<KeyFrame*> &lKeyFrames){
  bool Tcam_set = false;


  for(std::list<KeyFrame*>::const_iterator lit = lKeyFrames.begin(); lit != lKeyFrames.end(); lit++){
    KeyFrame* pKFi = *lit;
    if(pKFi->camera.camName == "Imaging"){

      if(!Tcam_set){ //Transform is the same for all imaging cameras so only create one vertex and pull prior estimate from any imaging keyframe
        g2o::VertexSE3* v_cam_transform = new g2o::VertexSE3();
        v_cam_transform->setEstimate( Converter::cvMatToIso3( pKFi->camera.Tcam.inv() ) );

        std::string vertex_name = "VertexSE3_Tcam";
        vertex_map.insert(std::make_pair(vertex_name, vertex_id));
        v_cam_transform->setId(vertex_id);
        vertex_id++;

        optimizer->addVertex(v_cam_transform);
        Tcam_set = true;
      }

      g2o::VertexTrajectoryTime* vt = new g2o::VertexTrajectoryTime();
      vt->setTrajectory(&trajectory);
      vt->setEstimate(g2o::Vector1d(pKFi->mTimeStamp));

      std::string vertex_name = "VertexTrajectoryTime" + std::to_string(pKFi->mnId);
      vertex_map.insert(std::make_pair(vertex_name, vertex_id));
      vt->setId(vertex_id);
      vertex_id++;
      optimizer->addVertex(vt);
    }
  }

}

void BundleAdjustment::SetImagingEdges(const std::list<KeyFrame*> &lKeyFrames){
  bool Tcam_set = false;

  for(std::list<KeyFrame*>::const_iterator lit = lKeyFrames.begin(); lit != lKeyFrames.end(); lit++){
    KeyFrame* pKFi = *lit;
//    std::cout << "pKFi->mnId: " << pKFi->mnId << std::endl;
    if(pKFi->camera.camName == "Imaging"){
      if(!Tcam_set){ //estiamted transform constraint
        g2o::EdgeTcam* eTcam = new g2o::EdgeTcam();
        std::string vertex_name ="VertexSE3_Tcam";
        eTcam->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_map[vertex_name]) ) );
        Isometry3 Tcam_est = Converter::cvMatToIso3( pKFi->camera.Tcam.inv() );
        eTcam->setMeasurement( Tcam_est );
        eTcam->setInformation(optParams.Info_ImagingTcam*g2o::Matrix6d::Identity() );
        eTcam->setLevel(0);
        optimizer->addEdge(eTcam);

        Tcam_set = true;
      }

      //measured time constraint
      g2o::EdgeTime* et = new g2o::EdgeTime();
      std::string vertex_name = "VertexTrajectoryTime" + std::to_string(pKFi->mnId);
      et->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name]) ) );
      g2o::Vector1d t_est = g2o::Vector1d(pKFi->mTimeStamp);
      et->setMeasurement(t_est);
      et->setInformation(optParams.Info_TrajTime*g2o::Matrix1d::Identity() );
      et->setLevel(0);
      optimizer->addEdge(et);

      //internal constraint - inferred imaging cam position must lie along trajectory.
      g2o::EdgeTrajectoryTimeTransformtoSE3* e_traj = new g2o::EdgeTrajectoryTimeTransformtoSE3();
      std::string vertex_name_0 = "VertexTrajectoryTime" + std::to_string(pKFi->mnId);
      std::string vertex_name_1 = "VertexSE3_Tcam";
      std::string vertex_name_2 = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
      e_traj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_0]) ) );
      e_traj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_1]) ) );
      e_traj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex( vertex_map[vertex_name_2]) ) );

      e_traj->setInformation(optParams.Info_TrajTimeSE3*g2o::Matrix7d::Identity() );
      e_traj->setLevel(0);
      optimizer->addEdge(e_traj);

    }
  }
}

std::vector<OutlierMono> BundleAdjustment::FindOutliersMono(const double thresh){
    std::vector<OutlierMono>  vpOutliersMono;

    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>thresh || !e->isDepthPositive())
        {
            OutlierMono o;
            o.e = e;
            o.pKF = vpEdgeKFMono[i];
            o.pMP = pMP;
            vpOutliersMono.push_back(o);
        }
    }

   return vpOutliersMono;
}

std::vector<OutlierStereo> BundleAdjustment::FindOutliersStereo(const double thresh){
    std::vector<OutlierStereo> vpOutliersStereo;

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>thresh || !e->isDepthPositive())
        {
            OutlierStereo o;
            o.e = e;
            o.pKF = vpEdgeKFStereo[i];
            o.pMP = pMP;
            vpOutliersStereo.push_back( o );
        }
    }

   return vpOutliersStereo;
}

bool BundleAdjustment::CheckForImagingCamera(const std::list<KeyFrame*> &lKeyFrames){
  bool img_cam_exists = false;
  for(std::list<KeyFrame*>::const_iterator lit = lKeyFrames.begin(); lit != lKeyFrames.end(); ++lit){
    if( (*lit)->camera.camName == "Imaging"){
      img_cam_exists = true;
      break;
    }
  }

  return img_cam_exists;
}


} //end ORBSLAM2 namespace
