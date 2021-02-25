#include "Trajectory_g2o.h"
#include <algorithm>
#include <cmath>

namespace g2o{

Trajectory::Trajectory(std::vector<Isometry3> poses_wc, std::vector<double> times, std::vector<bool> vtracking_good){
    std::vector<double>::iterator time_it = times.begin();
    std::vector<bool>::iterator good_it = vtracking_good.begin();

    Isometry3 Twc_prev;
    for(std::vector<Isometry3>::iterator iit = poses_wc.begin(); iit != poses_wc.end(); iit++, time_it++, good_it++){
        TrajectoryElement te;
        te.Twc = *iit;

        if(iit == poses_wc.begin())
        {
            Isometry3 V;
            V.setIdentity();
            te.Vwc = V;
        }
        else{
            te.Vwc = Twc_prev.inverse()*te.Twc; //cam to world- right multiply is forward in time
        }

        Twc_prev = *iit;

        te.time_stamp = *time_it;
        te.tracking_good = *good_it;

        trajectory_elements.push_back(te);

    }

}

void  Trajectory::clear(){
    trajectory_elements.clear();
}

TrajectoryElement Trajectory::back(){
    return trajectory_elements.back();
}

Isometry3 Trajectory::poseAtTime(double t){
    std::vector<TrajectoryElement>::iterator it;
    it = std::lower_bound(trajectory_elements.begin(), trajectory_elements.end(), t, compareTrajectoryElementTime);

    if(it ==trajectory_elements.begin()){ //time less than zero - shouldn't happen but could with optimization updates
        return (*it).Twc; //initial position
    }

    Isometry3 Vscaled;
    TrajectoryElement te_cur  = *it;
    TrajectoryElement te_prev = *(it-1);
    double dt_interval = te_cur.time_stamp - te_prev.time_stamp;
    double dt_target   = t - te_prev.time_stamp;
    scaleVelocity(te_cur.Vwc, dt_interval, dt_target, Vscaled);
    Isometry3 Tnow = te_prev.Twc * Vscaled;

    return Tnow;
}

Isometry3 Trajectory::velocityAtTime(double t){
    std::vector<TrajectoryElement>::iterator it;
    it = std::lower_bound(trajectory_elements.begin(), trajectory_elements.end(), t, compareTrajectoryElementTime);
    Isometry3 Vscaled; //LOTS OF DUPLICATED CODE WITH PREVIOUS FUNCTION - PULL OUT INTO SEPARATE FUNCTION _velocityAtTime(std::vector<TrajectoryElement>::iterator it, double time);
    TrajectoryElement te_cur  = *it;
    TrajectoryElement te_prev = *(it-1);
    double dt_interval = te_cur.time_stamp - te_prev.time_stamp;
    double dt_target   = t - te_prev.time_stamp;
    scaleVelocity(te_cur.Vwc, dt_interval, dt_target, Vscaled);

    return Vscaled;
}

bool compareTrajectoryElementTime(TrajectoryElement te, double time){
 if(te.time_stamp < time){
    return true;
  }
  else  {
     return false;
  }
}


void Trajectory::scaleVelocity(Isometry3 mVel, double vel_dt, double target_dt, Isometry3 &mVel_scaled){
    Eigen::Matrix4d Vscaled;
    Vscaled.setIdentity();

    double scale = std::abs(target_dt/vel_dt);
    Eigen::Vector3d t_v = mVel.translation().matrix();
    t_v = t_v * scale;  //dist of slam camera travelled over interval between mono imaging frames

    //determine rotation undergone by imaging camera, based on rotational velocity of slam camera
    Eigen::Quaternion<double> qbase(mVel.rotation().matrix());  //easiest to convert velocity to net rotation using quaternions
    double w_v = qbase.w();
    double ang_v = 2*acos(w_v);
    double ang_v_scaled = ang_v * scale;
    double w = cos(ang_v_scaled/2);
    Eigen::Vector3d axis;
    axis << qbase.x() , qbase.y() , qbase.z() ;
    axis.normalize();
    axis = sin(ang_v_scaled/2)*axis;
    Eigen::Quaternion<double> q_v(w, axis(0), axis(1), axis(2));
    Eigen::Matrix<double,3,3> Rv = q_v.toRotationMatrix();

    //assemble final matrix
    Vscaled.block<3,3>(0,0) = Rv;
    Vscaled.block<3,1>(0,3) = t_v;

    if((target_dt/vel_dt) < 0.0){
      mVel_scaled = Isometry3( Vscaled.inverse() );  //assume velocity was computed from forward camera motion so need to invert for going back in time
    }
    else{
      mVel_scaled = Isometry3( Vscaled );
    }

}

}
