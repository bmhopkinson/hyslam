

#include "Trajectory.h"
#include "Trajectory_g2o.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cmath>

using namespace ORB_SLAM2;

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


    //try eigen Isometry convertions and operations
    Trajectory traj;
    Isometry3 V0i = traj.cvMatToIso3(V0);
    Isometry3 V1i = traj.cvMatToIso3(V1);
    Isometry3 V2i = traj.cvMatToIso3(V2);
    Isometry3 V3i = traj.cvMatToIso3(V3);
    Isometry3 V4i = traj.cvMatToIso3(V4);
    Isometry3 V5i = traj.cvMatToIso3(V5);

    Isometry3 T0i = traj.cvMatToIso3(T0);
    Isometry3 T1i = T0i * V1i;
    Isometry3 T2i = T1i * V2i;
    Isometry3 T3i = T2i * V3i;
    Isometry3 T4i = T3i * V4i;
    Isometry3 T5i = T4i * V5i;

    std::cout << "cv::Mat T5: " << T5 << std::endl;
    std::cout << "Iso3 T5i:  " << T5i.matrix() << std::endl;

    std::vector<Isometry3> poses_wc;
    poses_wc.push_back(T0i); poses_wc.push_back(T1i); poses_wc.push_back(T2i);
    poses_wc.push_back(T3i); poses_wc.push_back(T4i); poses_wc.push_back(T5i);

    std::vector<double> times;
    times.push_back(time0); times.push_back(time1); times.push_back(time2);
    times.push_back(time3); times.push_back(time4); times.push_back(time5);

    std::vector<bool> vtracking_lost;
    vtracking_lost.push_back(false); vtracking_lost.push_back(false); vtracking_lost.push_back(false);
    vtracking_lost.push_back(false); vtracking_lost.push_back(false); vtracking_lost.push_back(false);

    g2o::Trajectory traj_g2o(poses_wc, times, vtracking_lost);
    Isometry3 Ttime = traj_g2o.poseAtTime(0.99);
    std::cout << "Ttime 0.99:" << Ttime.matrix() << std::endl;
    std::cout << "T1: " << T1 << std::endl;

    Ttime = traj_g2o.poseAtTime(2.99);
    std::cout << "Ttime 2.99:" << Ttime.matrix() << std::endl;
    std::cout << "T3: " << T3 << std::endl;

    Isometry3 Vtime = traj_g2o.velocityAtTime(2.99);
    std::cout << "Vtime 2.99:" << Vtime.matrix() << std::endl;
    std::cout << "V3: " << V3 << std::endl << std::endl;

    Isometry3 mVel(V1i);
    double target_dt = 0.5;
    double vel_dt = 1.0;

    Isometry3 mV2i_scaled;
    traj_g2o.scaleVelocity(V2i, vel_dt, target_dt, mV2i_scaled);
    std::cout <<"V2i scaled using scaleVelocity func: " << mV2i_scaled.matrix() << std::endl;

    double t_interp1 = 1.500;
    Isometry3 Tinterp1 = traj_g2o.poseAtTime(t_interp1);
    Isometry3 Tinterp1_target = T1i*mV2i_scaled;
    std::cout << "Tinterp1: " << Tinterp1.matrix() << std::endl;
    std::cout << "Tinterp1_target: " << Tinterp1_target.matrix() << std::endl << std::endl;

    double dt2 = 0.99;
    double t_interp2 = 3.00  + dt2;
    Isometry3 Tinterp2 = traj_g2o.poseAtTime(t_interp2);

    Isometry3 mV4i_scaled;
    traj_g2o.scaleVelocity(V4i, 1.00, dt2, mV4i_scaled);
    Isometry3 Tinterp2_target = T3i*mV4i_scaled;
    std::cout << "Tinterp2: " << Tinterp2.matrix() << std::endl;
    std::cout << "Tinterp2_target: " << Tinterp2_target.matrix() << std::endl;
    std::cout << "T3i:" << T3i.matrix() << std::endl;
    std::cout << "T4i:" << T4i.matrix() << std::endl;

    //edge case tests
    Isometry3 Tneg = traj_g2o.poseAtTime(-1.000);
    std::cout << "Tneg:" << Tneg.matrix() << std::endl;
    std::cout << "T0i:" << T0i.matrix() << std::endl;

    Isometry3 Tpos = traj_g2o.poseAtTime(10.000);
    std::cout << "Tpos:" << Tpos.matrix() << std::endl;
    std::cout << "T5i:" << T5i.matrix() << std::endl;
}
