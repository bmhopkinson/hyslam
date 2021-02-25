#include "SE3_sensor_edges.h"

namespace g2o {

using namespace std;


bool EdgeIMUQuat::read(std::istream& is){  // BH - NEED TO CHECK READ AND WRITE FUNCTIONS - JUST COPIED FROM ORBSLAM 
  for (int i=0; i<=4; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=3; i++)
    for (int j=i; j<=3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeIMUQuat::write(std::ostream& os) const {

  for (int i=0; i<=4; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=3; i++)
    for (int j=i; j<=3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

/*
void EdgeIMUQuat::linearizeOplus() {
   // HAD THIS WORKING FOR SO3 but SOMETHING'S NOT RIGHT FOR SE3
 	// see J. Sola Course on SLAM "Case of Unit Quaterion" page 45 eqn 4.65 for ALMOST CORRECT final equation  - there are some sign errors
	VertexSE3Expmap * v = static_cast<VertexSE3Expmap *>(_vertices[0]);
	//SE3Quat T(v->estimate());  
	Vector7d pose = v->estimate().toVector();
	Vector4d q    = pose.block<4,1>(3,0);
	
	std::cout << "q: " << q << std::endl;
	
	Matrix<double,4,3> SO3Jac = Matrix<double,4,3>::Zero();

	SO3Jac(0,0) = -0.5*q(1);
	SO3Jac(0,1) = -0.5*q(2);
	SO3Jac(0,2) = -0.5*q(3);
	
	SO3Jac(1,0) =  0.5*q(0);
	SO3Jac(1,1) =  0.5*q(3);
	SO3Jac(1,2) = -0.5*q(2);
	
	SO3Jac(2,0) = -0.5*q(3);
	SO3Jac(2,1) =  0.5*q(0);
	SO3Jac(2,2) =  0.5*q(1);
	
	SO3Jac(3,0) =  0.5*q(2);
	SO3Jac(3,1) = -0.5*q(1);
	SO3Jac(3,2) =  0.5*q(0);

	_jacobianOplusXi.block<4,3>(0,0) =  Matrix<double,4,3>::Zero();  
	_jacobianOplusXi.block<4,3>(0,3) =  SO3Jac;

		 
}
*/
}
