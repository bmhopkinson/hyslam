//
// Created by cv-bhlab on 7/20/21.
//

#ifndef HYSLAM_TRACKINGSTATETRANSITION_H
#define HYSLAM_TRACKINGSTATETRANSITION_H

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <Camera.h>
#include <InterThread.h>
#include <Initializer.h>
#include <FeatureFactory.h>


#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <map>


namespace HYSLAM {

using TrackingStateOptions = std::map<std::string, std::map<std::string, std::shared_ptr<TrackingState> > > ;

class TrackingStateTransition {
public:
    TrackingStateTransition(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                             std::ofstream &log, MainThreadsStatus* thread_status_,
                            FeatureFactory* factory);
    virtual void setInitialState(std::map<std::string, eTrackingState> &mState, std::map<std::string,
                                        std::shared_ptr<TrackingState> > &state)=0;
    virtual void transitionToNewState(std::map<std::string, eTrackingState> &mState, std::map<std::string,
                                    std::shared_ptr<TrackingState> > &state, bool bOK,  std::string cam_cur) = 0;

protected:
    virtual TrackingStateOptions loadStateOptions(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                                                                  std::ofstream &log, MainThreadsStatus* thread_status_, FeatureFactory* factory) =0;
    TrackingStateOptions   state_options;
};
} //end namespace

#endif //HYSLAM_TRACKINGSTATETRANSITION_H
