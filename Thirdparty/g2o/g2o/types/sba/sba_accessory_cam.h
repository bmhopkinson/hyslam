#ifndef G2O_SBA_ACCESSORY_CAM_H_
#define G2O_SBA_ACCESSORY_CAM_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "Trajectory_g2o.h"
//#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {

using namespace Eigen;
typedef Matrix<double, 1, 1> Vector1d;
typedef Matrix<double, 7, 1> Vector7d;

typedef Matrix<double, 1, 1> Matrix1d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 7, 7> Matrix7d;

class VertexTrajectoryTime : public BaseVertex <1, Vector1d>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexTrajectoryTime(){}
    void setToOriginImpl() {
        _estimate = Vector1d::Zero();
    }

    void oplusImpl(const number_t* v) {
      Vector1d update(v);
        _estimate += update;
    }
    
    bool read(std::istream& /*is*/) { return false; }
    bool write(std::ostream& /*os*/) const { return false; }


    void setTrajectory(Trajectory* tj){ _trajectory = tj; }
    Isometry3 poseAtCurrentTime();

  private:
    Trajectory* _trajectory;

};

class EdgeTrajectoryTimeTransformtoSE3 : public BaseMultiEdge<7, Vector7d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeTrajectoryTimeTransformtoSE3(){ 
       resize(3);
    }
    
    bool read(std::istream& /*is*/) { return false; }
    bool write(std::ostream& /*os*/) const { return false; }
    
    void computeError();
    //doesn't seem like we need a measurmement - and this edge is directly connected to a measurement it's an internal constraint
};


class EdgeTime: public  BaseUnaryEdge<1, Vector1d, VertexTrajectoryTime>
{  //constraint between estimated time and optimized time
  public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   EdgeTime(){}
   
    bool read(std::istream& /*is*/) { return false; }
    bool write(std::ostream& /*os*/) const { return false; }
   
   void computeError(){
       const VertexTrajectoryTime* v1 = static_cast<const VertexTrajectoryTime*>(_vertices[0]);
       Vector1d obs(_measurement);
       Vector1d est = v1->estimate();// might need to do type conversion here!!!!
       _error = est - obs ;
   }

};

class EdgeTcam: public  BaseUnaryEdge<6, Isometry3, VertexSE3>
{//constraint between estimated camera transform (relative to SLAM cam) and optimized camera transform
  public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   EdgeTcam(){  }
   
    bool read(std::istream& /*is*/) { return false; }
    bool write(std::ostream& /*os*/) const { return false; }
   
   void computeError(){
     VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);
     Isometry3 est = v->estimate();
     Isometry3 delta=_measurement.inverse() * est;
     _error = internal::toVectorMQT(delta);
   }
};

}
#endif
