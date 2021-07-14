#include "GlobalBundleAdjustment.h"
#include "OptHelpers.h"
#include <MapPointDB.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d_addons/SE3_sensor_edges.h"

#include<Eigen/StdVector>
#include<opencv2/core/eigen.hpp>

#include "Converter.h"

#include<mutex>
#include<vector>
#include<math.h>

namespace HYSLAM
{

int GlobalBundleAdjustment::ncalls = 0;  //initialize static member valiable

GlobalBundleAdjustment::GlobalBundleAdjustment(const std::vector<KeyFrame*> &vpKFfix_, const unsigned long nLoopKF_, int nIterations_, const bool bRobust_, bool* pbStopFlag_, Map* pMap_, g2o::Trajectory &trajectory_, optInfo optParams_) :
vpKFfix(vpKFfix_) , nLoopKF(nLoopKF_), nIterations(nIterations_), bRobust(bRobust_) , pbStopFlag(pbStopFlag_), BundleAdjustment( pMap_, trajectory_, optParams_)
{
   //setup optimizer and pass to BundleAdjustment base class
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;

    linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
          g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    setOptimizer(&optimizer);  //pass to base class BundleAdjustment

}


void GlobalBundleAdjustment::Run(){
	ncalls++;

    std::vector<KeyFrame*> vpAllKFs = pMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpAllMPs = pMap->GetAllMapPoints();

    //convert to list - probably should just swap all the BundleAdjustment functions to take vectors
    std::list<KeyFrame*> lAllKFs( vpAllKFs.begin(), vpAllKFs.end() );
    std::list<KeyFrame*> lFixedKFs( vpKFfix.begin(), vpKFfix.end() );
    std::list<MapPoint*>  lAllMPs( vpAllMPs.begin(), vpAllMPs.end() );


    std::list<KeyFrame*> lVariableKFs;
    for(std::list<KeyFrame*>::iterator lit = lAllKFs.begin(); lit != lAllKFs.end(); ++lit){
           KeyFrame* pKFi = *lit;

           bool variable = true;
           for(std::list<KeyFrame*>::iterator lit_f = lFixedKFs.begin(); lit_f != lFixedKFs.end(); ++lit_f){
               if(pKFi->mnId == (*lit_f)->mnId){
                   variable = false;
               }
           }
           if(variable){
                lVariableKFs.push_back(pKFi);
           }
    }  //end search for variable keyframes

  //set optimizer KeyFrame vertices
    SetKeyFrameVertices( lVariableKFs, false  );      // Set Local KeyFrame vertices
    SetKeyFrameVertices( lFixedKFs    , true  );       // Set Fixed KeyFrame vertices

    // set optimizer unary edges (sensor data -> keyframes)
    SetIMUEdges( lVariableKFs );         //IMU quaternion measurement constraint
    SetDepthEdges( lVariableKFs );      // add depth constraints
    SetGPSEdges( lVariableKFs ) ;  //add GPS constraints
    bool trackEdges = false;
    bool bRobust = false;
    SetMapPointVerticesEdges(  lAllMPs, trackEdges, bRobust );

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    RecoverOptimizedKeyFrames( lAllKFs );
    RecoverOptimizedMapPoints( lAllMPs );


} //end Run()


void GlobalBundleAdjustment::RecoverOptimizedKeyFrames( std::list<KeyFrame*> lKeyFrames)
{
  for(std::list<KeyFrame*>::iterator lit = lKeyFrames.begin(); lit != lKeyFrames.end(); ++lit){
      KeyFrame* pKFi = *lit;
      if(pKFi->isBad())
            continue;

        std::string vertex_name = "VertexSE3Expmap" + std::to_string(pKFi->mnId);
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(vertex_map[vertex_name] ));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(optParams.GBAtype == 1)
        {
            pKFi->SetPose(Converter::toCvMat(SE3quat));
        }
        else  //loop closing GBA, may need to handle newly added kfs so don't update yet
        {
            pKFi->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKFi->mTcwGBA);
            pKFi->mnBAGlobalForKF = nLoopKF;
        }
    }  //end loop on keyFrames
}


void GlobalBundleAdjustment::RecoverOptimizedMapPoints( std::list<MapPoint*> lMapPoints)
{
  for(std::list<MapPoint*>::iterator lit = lMapPoints.begin(); lit != lMapPoints.end(); ++lit){
       MapPoint* pMPi = *lit;
       //unsigned long id = pMPi->mnId+maxKFid+1;

        std::string vertex_name = "VertexSBAPointXYZ" + std::to_string(pMPi->mnId);
        if(sExcludedMPs.find( vertex_name ) != sExcludedMPs.end()) //test if mappoint is in the set that was excluded from optimization - if so, skip it
           continue;
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(vertex_map[vertex_name] ) );

        if(optParams.GBAtype == 1)
        {
            pMPi->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
           // pMPi->UpdateNormalAndDepth();
            pMap->update(pMPi);
        }
        else  //loop closing GBA
        {
            pMPi->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMPi->mPosGBA);
            pMPi->mnBAGlobalForKF = nLoopKF;
        }


  }  //end loop on MapPoints

}

} //end ORB_SLAM2 namespace
