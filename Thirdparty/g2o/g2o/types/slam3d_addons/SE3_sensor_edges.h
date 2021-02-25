
#ifndef G2O_SE3_SENSOR_EDGES
#define G2O_SE3_SENSOR_EDGES

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
//namespace SE3_IMU_edge {
//void init();
//}

using namespace Eigen;
typedef Matrix<double, 1, 1> Vector1d;
typedef Matrix<double, 4, 1> Vector4d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 6> Matrix6d;
/**
 *\ brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */

//class representing BNO255 IMU constraints which gives out absolute orientations in quaternions 
class  EdgeIMUQuat: public  BaseUnaryEdge<4, Vector4d, VertexSE3Expmap>{ // i think this is correct - < D, E, Vertex>; D - dimension of error (=dimension of measurment here), E - error type, Vertex - Vertex type
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeIMUQuat(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;
  
 // virtual void linearizeOplus();  // activate  when explict derivatives are available

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    
     Vector4d obs(_measurement);
     Vector7d est7 = v1->estimate().toVector();
     Vector4d est = est7.block<4,1>(3,0); // NOTE: .toVector() returns quat as is x,y,z,w 
     _error = est - obs ; 
     
  }

};
//placeholder for depth sensor constraints
class EdgeDepth: public  BaseUnaryEdge<1, Vector1d, VertexSE3Expmap>{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeDepth(){}
	
	virtual bool read(std::istream& /*is*/)
    {
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      return false;
    }
    
    void computeError(){
	 const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    
     Vector1d obs(_measurement);
     Vector7d est7 = v1->estimate().toVector();
	 Vector1d est = est7.block<1,1>(2,0); // z coordinate
     _error = est - obs ; 	
	}
	
};


// GPS class
class EdgeGPS: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	EdgeGPS(){}
	
	virtual bool read(std::istream& /*is*/)
    {
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      return false;
    }
    /*
    virtual void linearizeOplus(){ //turns out it's more complicated than this - this is correct for the x,y,z components but rotations come in as well - need to think about this
        _jacobianOplusXi = Matrix<double,3,6>::Zero(); /
        _jacobianOplusXi(0,0) = 1;
        _jacobianOplusXi(1,1) = 1;
        _jacobianOplusXi(2,2) = 1;
    }
 */
    void computeError(){
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    
        Vector3d obs(_measurement);
        Vector7d est7 = v1->estimate().toVector();
	    Vector3d est = est7.block<3,1>(0,0); // x, y, z coordinates
        _error = est - obs ; 	
	}
	
};

}

#endif
