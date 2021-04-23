#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
/*
 * represents path (per frame) of a camera as it travels in time through the world
 * key concept is that the underlying reference positions (KeyFrame poses) may change through optimization so must be sure to update trajectory regularly
 *
 * TrajectoryElement:
 *  data:
 *     Tcr - incremental transformation from ref keyframe to this frame; in world to camera format
 *     pRefKF - refrence keyframe. world position of camera can be found from Tcr and pRefKF->GetPose();
 *     Tcw - pose in world to camera convention
 *     Vcw - NOT SET PROPERLY RIGHT NOW - DO NOT USE - "Velocity" incremental transformation from previous frame's pose to this frame's pose (Twc); in world to camera format
 *     time data: time_stamp, dt - interval between previous frame and this frame
 *     tracking_good - boolean indicating if tracking was good when this frame was acquired
 *
 *  functionality: mostly straightforward except:
 *      update() - calculates current pose from Tcr and pRefKF->GetPose(), if current pRefKF is bad works back to find good KF and substitutes this as RefKF and changes Tcr to reflect new RefKF
 *
 * Trajectory:
 *   holds ordered vector of TrajectoryElements corresponding to each frame of a given camera, provide iterator functionality
 *   key functionality:
 *     int push_back(Frame &current_frame) - primary way new elements are added to trajectory - extracts info from frame and previous trajectory element (e.g. to compute dt)
 *     convertToG2O() - returns a g2o trajectory using eigen isometries for use in optimization
 *     integrateVelocity(const double t_start, const double t_stop, cv::Mat &Vint) - Vint is motion between t_start and t_stop in world to camera convention, uses _timeRange_() to get relevant subtrajectory.
 *              basic logic is scale velocities at beginning and end of time frame (t_start to t_stop) and simply compute one transform over interval that spans integer number of frames from first frame to last frame
 *              handles several edge cases (1: time frame is entirely between two frames; 2: time frame falls in interval that includes one frame but doesn't include entire frame interval)
 *     _timeRange_(const double t_start, const double t_stop, std::vector<TrajectoryElement> &subtraj)- subtraj includes all trajectory elements within the time range as well as one prior and one past, if available
 *
 */


#include <Frame.h>
#include <KeyFrame.h>

#include <opencv2/core/core.hpp>
#include "g2o/types/sba/Trajectory_g2o.h"

#include <vector>
#include <string>
#include <mutex>

//don't yet know how to make iterating thread safe so have a manual call to lock for use w/ iteration

//stores trajectory of camera - all frames
// can output trajectory in Isometry3 form for use with g2o
// Velocity class could probably be merged into this class

namespace HYSLAM{

//typedef Eigen::Transform<double ,3,Eigen::Isometry,Eigen::ColMajor> Isometry3;

class TrajectoryElement{
  public:
    TrajectoryElement();
    TrajectoryElement(Frame &frame);  //extract as much info as possible from frame but can't make a complete element
    TrajectoryElement(const TrajectoryElement &te);
    TrajectoryElement& operator=(const TrajectoryElement &te);
    void update();
    cv::Mat pose(); //in world to camera convention
    cv::Mat worldPosition(); // xyz position in world coordinates
    cv::Mat Tcr;  //incremental transformation from ref keyframe to this frame; in world to camera format
    KeyFrame* pRefKF;
    cv::Mat Tcw; //pose in world to camera convention
    cv::Mat Vcw; // "Velocity" incremental transformation from previous frame's pose to this frame's pose (Twc); in world to camera format
    double dt_vel; //time difference between frames
    double time_stamp;
    std::string name;
    bool tracking_good = false;
};

//consider adding a full trajectoryIterator class  : https://sourcemaking.com/design_patterns/iterator/cpp/1

class Trajectory{
  using Trajectory_t = std::vector<TrajectoryElement>;
public:
    Trajectory();
    void push_back(TrajectoryElement te);
    int push_back(Frame &current_frame);//, Frame &prev_frame);
    void updatePoses();
    std::vector<cv::Mat> getPoses(bool cam_to_world);
    void clear();
    bool empty() const {return trajectory_elements.empty(); };
    TrajectoryElement back() const;
    std::vector<TrajectoryElement> getTrajectoryElements() const {return trajectory_elements;  } //shouldn't need this
    bool getLastVelocityValid() const;

    g2o::Trajectory convertToG2O();
    int integrateVelocity(const double t_start, const double t_stop, cv::Mat &Vint) ; //Vint is motion between t_start and t_stop in world to camera convention
    int timeRange(const double t_start, const double t_stop, std::vector<TrajectoryElement> &subtraj) const;

    //iterator functionality
    using iterator = Trajectory_t::iterator;
    using const_iterator = Trajectory_t::const_iterator;
    iterator begin() {return trajectory_elements.begin(); }
    iterator end() {return trajectory_elements.end(); }
    const_iterator begin() const {return trajectory_elements.begin(); }
    const_iterator end() const {return trajectory_elements.end(); }
    const_iterator cbegin() const {return trajectory_elements.cbegin(); }
    const_iterator cend() const {return trajectory_elements.cend(); }

    mutable std::mutex trajectory_mutex;  //public so it can be locked for use with iteration
private:
    Trajectory_t trajectory_elements;
    std::vector<cv::Mat> poses;  //poses in world to camera convenction
    std::vector<cv::Mat> poses_wc;  //poses in camera to world convenction

    //internal non-locked functions
    void _push_back_(TrajectoryElement te);
    TrajectoryElement _back_() const;
    void _updatePoses_();

    cv::Mat _velocity_(TrajectoryElement &te_start, TrajectoryElement &te_stop);
    int _timeRange_(const double t_start, const double t_stop, std::vector<TrajectoryElement> &subtraj) const;

    std::vector<g2o::Isometry3> _getPosesIsometry_(bool cam_to_world);
    std::vector<double> _getTimes_() const ;
    std::vector<bool> _getTrackingGood_() const ;


 //   bool compareTrajectoryElementTime(TrajectoryElement* te, double time);
};

}

#endif
