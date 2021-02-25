#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "Trajectory.h"
#include "Converter.h"
#include "g2o/core/eigen_types.h"
#include "g2o/types/sba/Trajectory_g2o.h"
#include "g2o/types/sba/sba_accessory_cam.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace g2o;

typedef Eigen::Matrix<double, 1, 1> Matrix1d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

int main(){
    //V1-V5 are sucsessive camera motions in camera to world convention
    cv::Mat V0 = (cv::Mat_<float>(4,4) << 0.0, 0.0, 0.0, 0.00,         0.0, 0.0, 0.0, 0.0,           -0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat V1 = (cv::Mat_<float>(4,4) << 0.9659, 0.0, 0.2588, 0.50,   0.0, 1.0, 0.0, 1.0,          -0.2588, 0.0, 0.9659, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat V2 = (cv::Mat_<float>(4,4) << 1.0, 0.0, 0.0, 0.0,          0.0, 0.9659, -0.2588, 0.75,   0.0, 0.2588, 0.9659,-0.5,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat V3 = (cv::Mat_<float>(4,4) << 0.9848, -0.1736, 0.0, 0.50,  0.1736,  0.9848, 0.0, 0.0,    0.0, 0.0, 1.0, 1.0,         0.0, 0.0, 0.0, 1.0 );
    cv::Mat V4 = (cv::Mat_<float>(4,4) << 1.0, 0.0, 0.0, 0.5,          0.0, 0.9962, -0.0872, -0.25,  0.0, 0.0872, 0.9962, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat V5 = (cv::Mat_<float>(4,4) << 0.9659,-0.2588, 0.0, 1.0,    0.2588, 0.9659, 0.0, -2.0,    0.0, 0.0, 1.0, 0.5,         0.0, 0.0, 0.0, 1.0 );

    cv::Mat T0 = cv::Mat::eye(4,4,CV_32F);  //origin
    cv::Mat T1 = T0*V1;
    cv::Mat T2 = T1*V2;
    cv::Mat T3 = T2*V3;
    cv::Mat T4 = T3*V4;
    cv::Mat T5 = T4*V5;

 //   std::cout << "T2: " << T2 << std::endl;
    double time0 = 0.0;
    double time1 = 1.0;
    double time2 = 2.0;
    double time3 = 3.0;
    double time4 = 4.0;
    double time5 = 5.0;

    //eigen Isometry convertions and operations
    ORB_SLAM2::Trajectory traj;
    Isometry3 T0i = traj.cvMatToIso3(T0);
    Isometry3 T1i = traj.cvMatToIso3(T1);
    Isometry3 T2i = traj.cvMatToIso3(T2);
    Isometry3 T3i = traj.cvMatToIso3(T3);
    Isometry3 T4i = traj.cvMatToIso3(T4);
    Isometry3 T5i = traj.cvMatToIso3(T5);

    std::vector<Isometry3> poses_wc;
    poses_wc.push_back(T0i); poses_wc.push_back(T1i); poses_wc.push_back(T2i);
    poses_wc.push_back(T3i); poses_wc.push_back(T4i); poses_wc.push_back(T5i);

    std::vector<double> times;
    times.push_back(time0); times.push_back(time1); times.push_back(time2);
    times.push_back(time3); times.push_back(time4); times.push_back(time5);

    std::vector<bool> vtracking_lost;
    vtracking_lost.push_back(false); vtracking_lost.push_back(false); vtracking_lost.push_back(false);
    vtracking_lost.push_back(false); vtracking_lost.push_back(false); vtracking_lost.push_back(false);

    double time_est = 2.20;
    double time_true = 2.50;

    g2o::Trajectory traj_g2o(poses_wc, times, vtracking_lost);


    //optimizer
    g2o::SparseOptimizer* optimizer = new  g2o::SparseOptimizer();

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >  MyBlockSolver;
    typedef g2o::LinearSolverEigen<MyBlockSolver::PoseMatrixType> MyLinearSolver;
   // std::unique_ptr< MyBlockSolver::LinearSolverType> linearSolver;
     
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
         g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
    optimizer->setAlgorithm(solver);

    //VERTICES
    g2o::VertexTrajectoryTime* vt = new g2o::VertexTrajectoryTime();
    vt->setTrajectory(&traj_g2o);
    vt->setEstimate(Vector1d(time_est));
    vt->setId(0);

    g2o::VertexSE3* v_cam_transform = new g2o::VertexSE3();
    v_cam_transform->setEstimate(Isometry3::Identity());
    v_cam_transform->setId(1);

    g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
    v_pose->setEstimate(g2o::internal::toSE3Quat(T2i));
    v_pose->setId(2);

    optimizer->addVertex(vt);
    optimizer->addVertex(v_cam_transform);
    optimizer->addVertex(v_pose);

    //EDGES
    g2o::EdgeTime* et = new g2o::EdgeTime();
    et->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(0) ) );
    Vector1d vtime_true;
    vtime_true << time_true;
    et->setMeasurement(vtime_true);
    et->setInformation(Matrix1d::Identity() );
    et->setLevel(0);
    optimizer->addEdge(et);

    g2o::EdgeTcam* etcam = new g2o::EdgeTcam();
    etcam->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(1) ) );
    Isometry3 Tcam_true = Isometry3::Identity();
    Matrix<double, 3, 1> trans_v;
    trans_v << 1.0, 1.0, 2.0;
    Tcam_true.translation() = trans_v;
    
    etcam->setMeasurement( Tcam_true );
    etcam->setInformation(0.2*Matrix6d::Identity() );
    etcam->setLevel(0);
    optimizer->addEdge(etcam);

    g2o::EdgeTrajectoryTimeTransformtoSE3* e_traj = new g2o::EdgeTrajectoryTimeTransformtoSE3();
    e_traj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(0) ) );
    e_traj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(1) ) );
    e_traj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(2) ) );

    e_traj->setInformation(0.2*Matrix7d::Identity() );
    e_traj->setLevel(0);
    optimizer->addEdge(e_traj);
    
    //OPTIMIZE
    int nIterations = 5;
    optimizer->setVerbose(true);
    optimizer->initializeOptimization();
    optimizer->optimize(nIterations);
    
    // Recover optimized values
    g2o::VertexTrajectoryTime* vt_opt = static_cast<g2o::VertexTrajectoryTime*>(optimizer->vertex(0));
    Vector1d time_opt = vt_opt->estimate();
    std::cout << "optimize time: " << time_opt << "   ,time_true: " << time_true << std::endl;
    
    g2o::VertexSE3* v_cam_transform_opt = static_cast<g2o::VertexSE3*>(optimizer->vertex(1));
    Isometry3 cam_transform_opt = v_cam_transform_opt->estimate();
    std::cout << "cam_transform_opt: " << cam_transform_opt.matrix() << std::endl;
    
    g2o::VertexSE3Expmap* v_pose_opt = static_cast<g2o::VertexSE3Expmap*>(optimizer->vertex(2));
    g2o::SE3Quat SE3quat = v_pose_opt->estimate();
    std::cout << "pose_opt: " << ORB_SLAM2::Converter::toCvMat(SE3quat) << std::endl;
    Isometry3 pose_target = traj_g2o.poseAtTime(time_opt(0)) * Tcam_true;
    std::cout << " trajectory pose at time_opt: " << pose_target.matrix() << std::endl;

}
