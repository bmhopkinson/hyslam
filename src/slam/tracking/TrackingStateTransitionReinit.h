//
// Created by cv-bhlab on 7/21/21.
//

#ifndef HYSLAM_TRACKINGSTATETRANSITIONREINIT_H
#define HYSLAM_TRACKINGSTATETRANSITIONREINIT_H

#include <TrackingStateTransition.h>

namespace HYSLAM {

class TrackingStateTransitionReinit : public TrackingStateTransition {
public:
    TrackingStateTransitionReinit(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                                  std::ofstream &log, MainThreadsStatus* thread_status_,
                                  FeatureFactory* feature_factory);

    void setInitialState(std::map<std::string, eTrackingState> &mState, std::map<std::string,
                                std::shared_ptr<TrackingState> > &state);

    void transitionToNewState(std::map<std::string, eTrackingState> &mState, std::map<std::string, std::shared_ptr<TrackingState> > &state,
                              bool bOK,  std::string cam_cur);

protected:
    TrackingStateOptions loadStateOptions(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                             std::ofstream &log, MainThreadsStatus* thread_status_, FeatureFactory* factory);
    std::map< std::string, int> recent_init;
    std::map<std::string, Camera > camera_data;
};

}
#endif //HYSLAM_TRACKINGSTATETRANSITIONREINIT_H
