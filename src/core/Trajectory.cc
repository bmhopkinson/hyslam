#include <Trajectory.h>
#include <GenUtils.h>
#include <Converter.h>

#include <algorithm>

namespace HYSLAM{


TrajectoryElement::TrajectoryElement(){
  }

TrajectoryElement::TrajectoryElement(Frame &frame){
    if(frame.isTracked()) {
        Tcr = frame.mTcw * frame.mpReferenceKF->GetPoseInverse();
        Tcw = frame.mTcw.clone();
    } else {
        Tcr = cv::Mat::eye(4,4,CV_32F);
        Tcw = cv::Mat::eye(4,4,CV_32F);
    }
    pRefKF = frame.mpReferenceKF;
    Vcw = cv::Mat::eye(4,4,CV_32F);
    dt_vel = 0;
    time_stamp = frame.mTimeStamp;
    name = frame.fimgName;
    tracking_good = frame.isTracked();
}
TrajectoryElement::TrajectoryElement(const TrajectoryElement &te){
    Tcr= te.Tcr.clone();  //deep copy of cv::Mat
    pRefKF = te.pRefKF;
    Vcw = te.Vcw.clone(); // deep copy of cv::Mat
    dt_vel = te.dt_vel;
    time_stamp = te.time_stamp;
    name = te.name;
    tracking_good= te.tracking_good;
  }

TrajectoryElement& TrajectoryElement::operator=(const TrajectoryElement &te){
    if(this == &te)  //check for self assignment
      return *this;

    Tcr= te.Tcr.clone();  //deep copy of cv::Mat
    pRefKF = te.pRefKF;
    Vcw = te.Vcw.clone(); // deep copy of cv::Mat
    dt_vel = te.dt_vel;
    time_stamp = te.time_stamp;
    name = te.name;
    tracking_good= te.tracking_good;

    return *this;
}


void TrajectoryElement::update(){
    cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
    KeyFrame* pKF = pRefKF;
    while(pKF->isBad())
    {
        Trw = Trw*pKF->mTcp;
        pKF = pKF->GetParent();
    }

    Trw = Trw*pKF->GetPose();

    //update pose and possiblity pRefKF and transform
    Tcw = Tcr*Trw;
    if(pKF != pRefKF){
        pRefKF = pKF;
        Tcr = Tcw*pKF->GetPoseInverse();
    }

}

cv::Mat TrajectoryElement::pose(){
    update();
    return Tcw.clone();
}

cv::Mat TrajectoryElement::worldPosition(){
    update();
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc*tcw;
    return Ow.clone();
}

// TRAJECTORY MEMBER FUNCTIONS //
Trajectory::Trajectory(){}

//int Trajectory::push_back(Frame &current_frame, Frame &prev_frame){
void Trajectory::push_back(TrajectoryElement te) {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    _push_back_(te);
}

int Trajectory::push_back(Frame &current_frame){
    //general principal here is we're trying to extract as much valid info from the current and previous frame that's relevant to tracking as possible.

    std::lock_guard<std::mutex> lock(trajectory_mutex);
    TrajectoryElement te(current_frame);

    if(!empty()) {
        TrajectoryElement te_last = _back_();
        if( current_frame.isTracked() && te_last.tracking_good )
        {
            cv::Mat Tcr; //transform between current frame and valid refKF
            cv::Mat Tcw; //pose in world to camera convention
            KeyFrame* pRefKF;
            if(current_frame.mpReferenceKF->isBad()){
                KeyFrame* pKF = current_frame.mpReferenceKF;
                cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
                while(pKF->isBad())
                {

                    Trw = Trw*pKF->mTcp;
                    pKF = pKF->GetParent();
                }
                pRefKF = pKF;
                Trw = Trw*pKF->GetPose();
                Tcw = Trw;
                Tcr = current_frame.mTcw * Trw.inv();
            }
            else{
                pRefKF = current_frame.mpReferenceKF;
                Tcr = current_frame.mTcw*current_frame.mpReferenceKF->GetPoseInverse();
            }
            //velocity

            te_last.update();
            cv::Mat velCur = current_frame.mTcw * te_last.Tcw.inv();
            double velocity_dt = current_frame.mTimeStamp - te_last.time_stamp;

            te.Tcr = Tcr.clone();
            te.Tcw = Tcw.clone();
            te.pRefKF = pRefKF;
            te.Vcw = velCur.clone();
            te.dt_vel = velocity_dt;
        }
        else
        {
            te.dt_vel = current_frame.mTimeStamp - te_last.time_stamp;
        }

    }

    _push_back_(te);

    return 0;
}

void Trajectory::updatePoses(){
    //keyframe positions will be optimzed by bundle adjustment so need to periodically update frame poses
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    _updatePoses_();

}

void  Trajectory::clear(){
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory_elements.clear();
    poses.clear();
    poses_wc.clear();
}

TrajectoryElement Trajectory::back() const{
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    return _back_();
}

std::vector<cv::Mat> Trajectory::getPoses(bool cam_to_world) {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    _updatePoses_();

    if(cam_to_world){
        return poses_wc;
    }
    else {
        return poses;
    }

}

bool Trajectory::getLastVelocityValid() const {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    if(trajectory_elements.empty()){
       return false;
    }
    else{
       TrajectoryElement te = _back_();
      return te.tracking_good;
    }
}

g2o::Trajectory Trajectory::convertToG2O(){
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    std::vector<g2o::Isometry3> poses_wc_i3 = _getPosesIsometry_(true);
    std::vector<double> times = _getTimes_();
    std::vector<bool> vtracking_good = _getTrackingGood_();

    return g2o::Trajectory(poses_wc_i3, times, vtracking_good);
}



int Trajectory::integrateVelocity(const double t_start, const double t_stop, cv::Mat &Vint) {
  cv::Mat vel_temp = cv::Mat::eye(4,4, CV_32F);
  cv::Mat v_alt; //alternate method's velocity
  //provides transformation from position at time t_start to position at time t_stop
  // works both forward and backward in time (but of course only over motion that has already occured).

  bool invert = false;
  double _t_start, _t_stop;

  if(t_start > t_stop){  //motion backward in time
      invert = true;
      _t_start = t_stop;
      _t_stop  = t_start;
  }
  else {
      _t_start = t_start;
      _t_stop  = t_stop;
  }

  std::vector<TrajectoryElement> subtraj;
  if(_timeRange_(_t_start, _t_stop,  subtraj) == 0){
      //do integration

      //ALTERNATE - More correct but admittely confusing method
      //basic logic is scale velocities at beginning and end of time frame (t_start to t_stop)
      // and simply compute one transform over interval that spans integer number of frames from first frame to last frame
      // have to handle edge cases first (1: time frame is entirely between two frames; 2: time frame falls in interval that includes one frame but doesn't include entire frame interval)
      // then handle more general case
      //First calculate scaled veloctiy between element 0 and 1
      std::vector<TrajectoryElement>::iterator vit = subtraj.begin();
      TrajectoryElement& te0 = *vit;
      vit++;  //there must be at least two elements in the subtrajectory - so this should produce a valid trajectory element
      TrajectoryElement& te1 = *vit;
      cv::Mat v01 = _velocity_(te0, te1);
      cv::Mat v01_scaled;
      double dt_vel = te1.time_stamp - te0.time_stamp;

      double dt_target;
      if(_t_stop <te1.time_stamp ){
          dt_target = _t_stop - _t_start;//t_start to t_stop falls within one frame interval
      } else {
          dt_target = te1.time_stamp - _t_start;
      }

      GenUtils::ScaleVelocity(v01, dt_vel, dt_target, v01_scaled);

      if(std::next(vit,1) == subtraj.end() ){  //t_start to t_stop falls within one frame interval
        v_alt = v01_scaled;
      } else {  // calculate scaled velocity between element n-1 and n (n = last element)
          std::vector<TrajectoryElement>::reverse_iterator rit = subtraj.rbegin();
          TrajectoryElement& teN = *rit;
          rit++; //step back
          TrajectoryElement& teNm1 = *rit;
          cv::Mat vNm1N = _velocity_(teNm1, teN);
          cv::Mat vNm1N_scaled;
          dt_vel = teN.time_stamp - teNm1.time_stamp;
          dt_target = _t_stop - teNm1.time_stamp;
          GenUtils::ScaleVelocity(vNm1N, dt_vel, dt_target, vNm1N_scaled);

          if(&te1 == &teNm1) {  //time interval spans only two frame intervals
              v_alt = vNm1N_scaled * v01_scaled;
          } else {  //general case
              cv::Mat v_interval = _velocity_(te1, teNm1);
              v_alt = vNm1N_scaled * v_interval *  v01_scaled;
          }

      }
  //    std::cout << "vel_temp: " << vel_temp << std::endl;
  //    std::cout << "v_alt: " << v_alt << std::endl;
  }
  else {
      return -1;
  }

  if(invert){
  //    vel_temp = vel_temp.inv();
      v_alt = v_alt.inv();
  }

  //Vint = vel_temp.clone();
  Vint = v_alt.clone();
  return 0;

}

int Trajectory::timeRange(const double t_start, const double t_stop, std::vector<TrajectoryElement> &subtraj) const
{   // t_start must be oldest time, t_stop most recent;
    //includes all trajectory elements within the time range as well as one prior and one past, if available
    //note: the motion from times(i) to times(i+1) is described by vels(i+1) - consequently vels[0] is not relevant
    // returns 0 if succesful, -1 if time range requested exceeds trajectory's span
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    return _timeRange_(t_start, t_stop, subtraj);
}

//  PRIVATE MEMBER FUNCTIONS

void Trajectory::_push_back_(TrajectoryElement te) {
    trajectory_elements.push_back(te);
}

TrajectoryElement Trajectory::_back_() const{
    return trajectory_elements.back();
}

void Trajectory::_updatePoses_(){
    //keyframe positions will be optimzed by bundle adjustment so need to periodically update frame poses

    std::vector<cv::Mat> updated_poses;
    std::vector<cv::Mat> updated_poses_wc;

    for(std::vector<TrajectoryElement>::iterator vit=trajectory_elements.begin(); vit != trajectory_elements.end(); ++vit)
    {

        vit->update();
        cv::Mat Tcw = vit->Tcw;
        updated_poses.push_back(Tcw);
        updated_poses_wc.push_back(Tcw.inv());

    }

    poses = updated_poses;
    poses_wc = updated_poses_wc;

}

cv::Mat Trajectory::_velocity_(TrajectoryElement &te_start, TrajectoryElement &te_stop){
    te_start.update();
    te_stop.update();
    cv::Mat V = te_stop.Tcw * te_start.Tcw.inv();
    return V.clone();
}

int Trajectory::_timeRange_(const double t_start, const double t_stop, std::vector<TrajectoryElement> &subtraj) const
{   // t_start must be oldest time, t_stop most recent;
    //includes all trajectory elements within the time range as well as one prior and one past, if available
    //note: the motion from times(i) to times(i+1) is described by vels(i+1) - consequently vels[0] is not relevant
    // returns 0 if succesful, -1 if time range requested exceeds trajectory's span

    TrajectoryElement te_prev;
    bool prev_valid = false;

    bool keep = false;

    std::vector<TrajectoryElement>::const_iterator it = trajectory_elements.begin();
    if((*it).time_stamp > t_start){ //requested start time is prior to start of trajectory
        return -1;
    }

    for(; it != trajectory_elements.end(); it++)
    {
        TrajectoryElement te = *it;
        double t_cur = te.time_stamp;

        if(keep){ //append data
            subtraj.push_back( te );
        }
        else if( t_start <= t_cur ){  //start appending data
            keep = true;
            if(prev_valid){
                subtraj.push_back( te_prev );
                subtraj.push_back( te );
            }
            else{  //requested time earlier than start of trajectory
                return -1;
            }
        }

        if(t_cur >= t_stop){  //stop appending data
            keep = false;
            break;
        }

        //store previous values
        te_prev = te;
        prev_valid = true;

    }

    if(it == trajectory_elements.end()){  // requested range exceeded available.
        return -1;
    }
    else
    {
        return 0;
    }
}

std::vector<double> Trajectory::_getTimes_() const {
    std::vector<double> times;

    for(std::vector<TrajectoryElement>::const_iterator it = trajectory_elements.begin(); it != trajectory_elements.end(); it++){
        times.push_back( (*it).time_stamp );
    }

    return times;
}

std::vector<bool> Trajectory::_getTrackingGood_() const {
    std::vector<bool> tracking_good;

    for(std::vector<TrajectoryElement>::const_iterator it = trajectory_elements.begin(); it != trajectory_elements.end(); it++){
        tracking_good.push_back( (*it).tracking_good );
    }

    return tracking_good;
}

std::vector<g2o::Isometry3> Trajectory::_getPosesIsometry_(bool cam_to_world)
{
    _updatePoses_();
    std::vector<g2o::Isometry3> tj;

    std::vector<cv::Mat>* poses_sel;
    if(cam_to_world){
        poses_sel = &poses_wc;
    }
    else{
        poses_sel = &poses;
    }

    for(std::vector<cv::Mat>::iterator it = (*poses_sel).begin(); it != (*poses_sel).end(); it++){
        cv::Mat Tcv = *it;

        g2o::Isometry3 Ti3 = Converter::cvMatToIso3(Tcv);
        tj.push_back(Ti3);
    }

    return tj;

}

}
