
#include "Tracking_datastructs.h"
#include "Converter.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iterator>
#include <cmath>
#include <algorithm>

namespace HYSLAM
{

TrackLocalMapParameters::TrackLocalMapParameters(){}

TrackLocalMapParameters::TrackLocalMapParameters(cv::FileNode data){
    match_nnratio = data["match_nnratio"];
    match_radius_threshold = data["match_radius_threshold"];
    N_max_local_keyframes = data["N_max_local_keyframes"];
    N_neighbor_keyframes = data["N_neighbor_keyframes"];
}


TrackMotionModelParameters::TrackMotionModelParameters(){}

TrackMotionModelParameters::TrackMotionModelParameters(cv::FileNode data){
    N_min_matches = data["N_min_matches"];
    match_nnratio = data["match_nnratio"];
    match_radius_threshold_stereo = data["match_radius_threshold_stereo"];
    match_radius_threshold_other = data["match_radius_threshold_other"];
    match_theshold_inflation_factor = data["match_theshold_inflation_factor"];
}

TrackPlaceRecognitionParameters::TrackPlaceRecognitionParameters(){
 setDefaultRANSAC();
}

TrackPlaceRecognitionParameters::TrackPlaceRecognitionParameters(cv::FileNode data){
    setDefaultRANSAC();
    match_nnratio_1 = data["match_nnratio_1"];
    match_nnratio_2 = data["match_nnratio_2"];
    N_min_matches_BoW = data["N_min_matches_BoW"];
    PnPsolver_iterations = data["PnPsolver_iterations"];
    N_min_matches_PoseOpt = data["N_min_matches_PoseOpt"];
    N_min_matches_success = data["N_min_matches_success"];
    match_radius_threshold_1 = data["match_radius_threshold_1"];
    match_radius_threshold_2 = data["match_radius_threshold_2"];
    ORBdist_1 = data["ORBdist_1"];
    ORBdist_2 = data["ORBdist_2"];
}

void TrackPlaceRecognitionParameters::setDefaultRANSAC(){
    ransac.probability =  0.99;
    ransac.minInliers = 10;
    ransac.maxIterations = 300;
    ransac.minSet = 4;
    ransac.epsilon = 0.5;
    ransac.th2 = 5.991;
}


TrackReferenceKeyFrameParameters::TrackReferenceKeyFrameParameters(){}

TrackReferenceKeyFrameParameters::TrackReferenceKeyFrameParameters(cv::FileNode data){
    match_nnratio = data["match_nnratio"];
    N_min_matches_BoW = data["N_min_matches_BoW"];

}


// STATES

StateNormalParameters::StateNormalParameters(){}

StateNormalParameters::StateNormalParameters(cv::FileStorage data){
    cv::FileNode state_data = data["States"]["Normal"];
    cv::FileNode config_data_strategies =  data["Strategies"];

    tmomo_params = TrackMotionModelParameters(config_data_strategies[ state_data["Strategies"]["TrackMotionModel"].string() ]);
    trefkf_params = TrackReferenceKeyFrameParameters(config_data_strategies[ state_data["Strategies"]["TrackReferenceKeyFrame"].string() ]);
    tlocalmap_params =  TrackLocalMapParameters(config_data_strategies[ state_data["Strategies"]["TrackLocalMap"].string()  ]);

    thresh_init = state_data["thresh_init"]; //number of tracked points initially needed
    thresh_refine = state_data["thresh_refine"];

    N_tracked_target = state_data["N_tracked_target"];
    N_tracked_variance = state_data["N_tracked_variance"];
    min_KF_interval = state_data["min_KF_interval"];
    max_KF_interval = state_data["max_KF_interval"];
    min_N_tracked_close = state_data["min_N_tracked_close"];
    thresh_N_nontracked_close = state_data["thresh_N_nontracked_close"];
    min_frac_refKF_mono = state_data["min_frac_refKF_mono"];
    min_frac_refKF_stereo = state_data["min_frac_refKF_stereo"];

}


StateInitializeParameters::StateInitializeParameters(){}

StateInitializeParameters::StateInitializeParameters(cv::FileStorage data){
    cv::FileNode state_data = data["States"]["Initialize"];
    cv::FileNode config_data_strategies =  data["Strategies"][ state_data["Strategies"]["Initialize"].string() ];

    mono_params = MonoInitializerParameters(config_data_strategies["Mono"]);
    stereo_params = StereoInitializerParameters(config_data_strategies["Stereo"]);

    N_min_mpts_monoslam = state_data["N_min_mpts_monoslam"];
}


StateRelocalizeParameters::StateRelocalizeParameters(){}

StateRelocalizeParameters::StateRelocalizeParameters(cv::FileStorage data){
    cv::FileNode state_data = data["States"]["Relocalize"];
    cv::FileNode strategies =  data["Strategies"];

    tplacerecog_params = TrackPlaceRecognitionParameters(strategies[ state_data["Strategies"]["TrackPlaceRecognition"].string() ]) ;
    tlocalmap_params = TrackLocalMapParameters( strategies[ state_data["Strategies"]["TrackLocalMap"].string() ] ) ;

    thresh_init = state_data["thresh_init"]; //number of tracked points initially needed
    thresh_refine = state_data["thresh_refine"]; //number of tracked points required after refinement for success
}

void Velocity::addElement(const cv::Mat vel, double &time, bool valid)
{
    if(velocities.size() == max_size){
        velocities.pop_back();
        time_stamps.pop_back();
    }
    velocities.push_front(vel.clone());
    time_stamps.push_front(time);
    last_valid = valid;

}

int Velocity::timeRange(const double t_start, const double t_stop, std::vector<cv::Mat> &vels, std::vector<double> &times)
{   // t_start must be oldest time, t_stop most recent;
    //includes all vels within the time range as well as one prior and one past, if available
    // output vels and times begin at (or near) t_start and end at (or near) t_stop
    //note: the motion from times(i) to times(i+1) is described by vels(i+1) - consequently vels[0] is not relevant
    // returns 0 if succesful, -1 if t_stop exceeds available data buffer

    //procedure is slightly confusing b/c time_stamps and velocities are stored with most recent values first
    std::deque<double>::const_iterator it_t = time_stamps.begin();
    std::deque<cv::Mat>::const_iterator it_vel = velocities.begin();

    double t_prev = *it_t;
    cv::Mat vel_prev = *it_vel;
    bool prev_valid = false;

    bool keep = false;

    for( ; it_t != time_stamps.end(); ++it_t, ++it_vel){
        double t_cur = *it_t;

        if(keep){ //append data
            vels.push_back( (*it_vel).clone() );
            times.push_back(t_cur);
        }
        else if( t_cur <= t_stop ){  //start appending data
            keep = true;
            if(prev_valid){
                vels.push_back( vel_prev.clone() );
                vels.push_back( (*it_vel).clone() );

                times.push_back(t_prev);
                times.push_back(t_cur);
            }
            else{  //occurs when most recent velocity data (element 0) is included in time range
                vels.push_back( (*it_vel).clone() );
                times.push_back(t_cur);
            }
        }

        if(t_cur <= t_start){  //stop appending data
            keep = false;
            break;
        }

        //store previous values
        t_prev = t_cur;
        vel_prev = *it_vel;
        prev_valid = true;

    }

    //reverse order of vels and times so that values start at t_start and end at t_stop
    std::reverse(vels.begin() , vels.end() );
    std::reverse(times.begin(), times.end());

    if(it_t == time_stamps.end()){  // requested range exceeded available.
        return -1;
    }
    else
    {
        return 0;
    }

}

int Velocity::integrate(const double t_start, const double t_stop, cv::Mat &vel_integrated){
    cv::Mat vel_temp = cv::Mat::eye(4,4, CV_32F);
    //provides transformation from position at tiint integrateVelocity(const double t_start, const double t_stop, cv::Mat &Vint);me t_start to position at time t_stop
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

    std::vector<cv::Mat> vels;
    std::vector<double> times;
    if(timeRange(_t_start, _t_stop,  vels, times) == 0){
        //do integration
        for(std::vector<cv::Mat>::const_iterator vit = vels.begin()+1; vit != vels.end(); ++vit) //starts at 2nd velocity element b/c first is not relevant (see timeRange doc)
        {
            cv::Mat v = *vit;
            if(v.empty())
            {
                return -1; //currently fails if tracking was lost at any point during the time range
            }
            else if (vit == vels.begin()+1){ //first element - needs to be scaled
                cv::Mat v_scaled;
                double dt_vel = times[1] - times[0];
                double dt_target = times[1] - _t_start;
         //       std::cout << "dt_vel: " << dt_vel << " , dt_target: " << dt_target << std::endl;
                ScaleVelocity(v, dt_vel, dt_target, v_scaled);
          //      std::cout << "1st element scaled: " << v_scaled << std::endl;
                v = v_scaled;
            }
            else if(std::next(vit,1) == vels.end()){ //last element- needs to be scaled
                cv::Mat v_scaled;
                double dt_vel = *(times.rbegin()) - *(times.rbegin()+1);
                double dt_target = _t_stop - *(times.rbegin()+1);
            //    std::cout << "dt_vel: " << dt_vel << " , dt_target: " << dt_target << std::endl;
                ScaleVelocity(v, dt_vel, dt_target, v_scaled);
                v = v_scaled;
            //    std::cout << "last element scaled: " << v_scaled << std::endl;

            }
            vel_temp = v*vel_temp;
        }

    }
    else {
        return -1;
    }

    if(invert){
        vel_temp = vel_temp.inv();
    }

    vel_integrated = vel_temp.clone();
    return 0;

}
cv::Mat Velocity::operator[](const int i){
    return velocities[i].clone();
}

void Velocity::ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled){
    cv::Mat mVwc = cv::Mat::eye(4,4,CV_32F);
    double scale = std::abs(target_dt/vel_dt);
    cv::Mat mVelwc = mVel.inv();
    cv::Mat t_v = mVelwc.rowRange(0,3).col(3);
    cv::Mat t_v_scaled = t_v * scale;  //dist of slam camera travelled over interval between mono imaging frames

    //determine rotation undergone by imaging camera, based on rotational velocity of slam camera
    cv::Mat Rbase = mVelwc.rowRange(0,3).colRange(0,3);
    Eigen::Matrix<double,3,3> Rbase_E = Converter::toMatrix3d(Rbase);
    Eigen::Quaternion<double> qbase(Rbase_E);  //easiest to convert velocity to net rotation using quaternions
    double w_v = qbase.w();
    double ang_v = 2*acos(w_v);
    double ang_v_scaled = ang_v * scale;
    double w = cos(ang_v_scaled/2);
    Eigen::Vector3d axis;
    axis << qbase.x() , qbase.y() , qbase.z() ;
    axis.normalize();
    axis = sin(ang_v_scaled/2)*axis;
    Eigen::Quaternion<double> q_v(w, axis(0), axis(1), axis(2));
    Eigen::Matrix<double,3,3> Rv_E = q_v.toRotationMatrix();

    //assemble final matrix
    cv::Mat Rv = Converter::toCvMat(Rv_E);
    cv::Mat Tv = cv::Mat::eye(4,4,CV_32F);
    Rv.copyTo( mVwc.rowRange(0,3).colRange(0,3) );
    t_v_scaled.copyTo(mVwc.rowRange(0,3).col(3) );

    if((target_dt/vel_dt) < 0.0){
      mVwc = mVwc.inv();  //assume velocity was computed from forward camera motion so need to invert for going back in time
    }

    mVel_scaled = mVwc.clone();
    mVel_scaled = mVel_scaled.inv();

}




void MonoInitialMatch::clear(){
    mvIniMatches.clear();
    mvbPrevMatched.clear();
    mvIniP3D.clear();
    mInitialFrame = Frame();
}


}
