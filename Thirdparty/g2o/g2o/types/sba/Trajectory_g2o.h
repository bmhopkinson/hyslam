#ifndef TRAJECTORY_G2O_H_
#define TRAJECTORY_G2O_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

//stores trajectory of camera - all frames and times for use in g2o optimization
namespace g2o{

typedef Eigen::Transform<double ,3,Eigen::Isometry,Eigen::ColMajor> Isometry3;

struct TrajectoryElement{
    Isometry3 Vwc;  //incremental transformation from previous keyframe to this pose (Twc); in camera to world format
    Isometry3 Twc;  //pose in camera to world format
    double time_stamp;
    bool tracking_good;
};

bool compareTrajectoryElementTime(TrajectoryElement te, double time);

class Trajectory{
public:
    Trajectory();
    Trajectory(std::vector<Isometry3> poses_wc, std::vector<double> times, std::vector<bool> vtracking_good);
    void push_back(TrajectoryElement te){ trajectory_elements.push_back(te); };
    void clear();
    TrajectoryElement back();
    std::vector<TrajectoryElement> getTrajectoryElements() {return trajectory_elements;  }
    Isometry3 poseAtTime(double t);
    Isometry3 velocityAtTime(double t);
    void scaleVelocity(Isometry3 mVel, double vel_dt, double target_dt, Isometry3 &mVel_scaled);       //make private after done testing

private:
    std::vector<TrajectoryElement> trajectory_elements;



};

}

#endif
