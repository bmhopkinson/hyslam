#include "sba_accessory_cam.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o{

Isometry3 VertexTrajectoryTime::poseAtCurrentTime(){
    return _trajectory->poseAtTime(_estimate(0)); //trajectory should return isometry3 using Eigen structures not opencv - don't want to have to compile g2o with opencv
}

void EdgeTrajectoryTimeTransformtoSE3::computeError()
{
    VertexTrajectoryTime* time = static_cast<VertexTrajectoryTime *>(_vertices[0]);
    VertexSE3* T_slam_to_img = static_cast<VertexSE3 *>(_vertices[1]);
    VertexSE3Expmap* vSE3 = static_cast<VertexSE3Expmap *>(_vertices[2]);

    Isometry3 Tslam_wc = time->poseAtCurrentTime(); //camera to world convetion
    Isometry3 Tslam = Tslam_wc.inverse(); //world to camera convention
    Isometry3 T_se3 = T_slam_to_img->estimate();
    Isometry3 T_acc = T_se3* Tslam ; //or similar
    Eigen::Vector3d trans = T_acc.translation().matrix();
    Eigen::Quaternion<double> q_acc(T_acc.rotation().matrix());
    Vector7d t_acc_v7;
    t_acc_v7 << trans, q_acc.x(), q_acc.y(), q_acc.z(), q_acc.w();

    Vector7d est7 = vSE3->estimate().toVector();

    _error = est7 - t_acc_v7; //refine - but fine to start

}

}//end g2o namespace
