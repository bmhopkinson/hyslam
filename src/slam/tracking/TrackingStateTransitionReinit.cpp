//
// Created by cv-bhlab on 7/21/21.
//

#include <TrackingStateTransitionReinit.h>
#include <TrackingStateNormal.h>
#include <TrackingStateRelocalize.h>
#include <TrackingStateInitialize.h>
#include <TrackingStatePostInitialization.h>
#include <TrackingStateReInitialize.h>
#include <TrackingStateNull.h>


namespace HYSLAM{

TrackingStateTransitionReinit::TrackingStateTransitionReinit(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                                                               std::ofstream &log, MainThreadsStatus* thread_status_,
                                                             FeatureFactory* feature_factory):
        TrackingStateTransition(config_data, cam_data, optimizer_info_, init_data_, log, thread_status_, feature_factory){

    state_options = loadStateOptions(config_data, cam_data, optimizer_info_, init_data_,  log, thread_status_, feature_factory);
    camera_data = cam_data;

}

TrackingStateOptions TrackingStateTransitionReinit::loadStateOptions(cv::FileStorage config_data, const std::map<std::string, Camera > &cam_data, optInfo optimizer_info_, std::map< std::string, InitializerData> &init_data_,
                                                                      std::ofstream &log, MainThreadsStatus* thread_status_, FeatureFactory* feature_factory) {
    TrackingStateOptions options;
    cv::FileNode cam_states = config_data["Cameras"];
    cv::FileNode state_config = config_data["States"];
    cv::FileNode strategy_config = config_data["Strategies"];

    for(auto it = cam_data.begin(); it != cam_data.end(); ++it) {
        std::string cam_name = it->first;
        Camera cam = it->second;
        //Initialization
        StateInitializeParameters state_initialize_params(state_config[cam_states[cam_name]["Initialize"].string()],
                                                          strategy_config);
        options[cam_name]["INITIALIZATION"] = std::make_shared<TrackingStateInitialize>(optimizer_info_, cam, init_data_[cam.camName], state_initialize_params,
                                                                                              log, thread_status_, feature_factory);

        //Normal
        StateNormalParameters state_normal_params(state_config[cam_states[cam_name]["Normal"].string()], strategy_config);
        options[cam_name]["NORMAL"] = std::make_shared<TrackingStateNormal>(optimizer_info_, state_normal_params,
                                                                                  log, thread_status_, feature_factory);
        //post initialization - same as normal but force keyframe creation
        options[cam_name]["POSTINIT"] = std::make_shared<TrackingStatePostInitialization>(optimizer_info_, state_normal_params,
                                                                                                log, thread_status_, feature_factory);

        //Relocalize
        StateRelocalizeParameters state_relocalize_params(state_config[cam_states[cam_name]["Relocalize"].string()],
                                                          strategy_config);
        options[cam_name]["RELOCALIZE"] = std::make_shared<TrackingStateRelocalize>(optimizer_info_, state_relocalize_params,
                                                                                          log, thread_status_, feature_factory);

        StateReInitializeParameters state_reinitialize_params(state_config[cam_states[cam_name]["ReInitialize"].string()],
                                                              strategy_config);
        options[cam_name]["REINITIALIZE"] = std::make_shared<TrackingStateReInitialize>(optimizer_info_, cam, init_data_[cam.camName],
                                                                                              state_reinitialize_params,  log, thread_status_, feature_factory);

        //Null
        options[cam_name]["NULL"] = std::make_shared<TrackingStateNull>(log, thread_status_);
    }


    return options;
}

void TrackingStateTransitionReinit::setInitialState(std::map<std::string, eTrackingState> &mState,
                                                    std::map<std::string, std::shared_ptr<TrackingState>> &state) {
    for(auto it = camera_data.begin(); it != camera_data.end(); ++it) {
        std::string cam_name = it->first;
        mState[cam_name] = eTrackingState::INITIALIZATION;
        state[cam_name] = state_options[cam_name]["INITIALIZATION"];
    }

}

void TrackingStateTransitionReinit::transitionToNewState(std::map<std::string, eTrackingState> &mState, std::map<std::string,
        std::shared_ptr<TrackingState> > &state, bool bOK, std::string cam_cur)
{
    eTrackingState next_state = mState[cam_cur]; //assume we stay in the same state
    std::shared_ptr<TrackingState> pnext_track_state = state[cam_cur];

    if(mState[cam_cur] == eTrackingState::INITIALIZATION){
        if(bOK){
            state[cam_cur]->clear();  //be sure to clear initializer!
            next_state = eTrackingState::POSTINIT;
            pnext_track_state = state_options[cam_cur]["POSTINIT"];
            recent_init[cam_cur] = 5;
        }
    } else if (mState[cam_cur] == eTrackingState::POSTINIT){
        if( recent_init[cam_cur] > 0 ){
            recent_init[cam_cur]--;
        } else{
            next_state = eTrackingState::NORMAL;
            pnext_track_state = state_options[cam_cur]["NORMAL"];
        }
    }
    else if(mState[cam_cur] == eTrackingState::NORMAL) {
/*
        if((mCurrentFrame.mnId % 300) == 0 && mCurrentFrame.mnId>200){
            if (cam_cur == "SLAM") {
                pnext_track_state = state_options[cam_cur]["REINITIALIZE"];
                next_state = eTrackingState::REINITIALIZE;
                std::cout << "SLAM CAMERA LOST TRACKING, TRYING TO REINITIALIZE: frameid: " << mCurrentFrame.mnId << ", name:" << mCurrentFrame.fimgName <<  std::endl;

                if(state.find("Imaging") != state.end()){ //if there's an imaging camera set it to NULL state b/c we can't track it if SLAM tracking is lost
                    state["Imaging"]  = state_options["Imaging"]["NULL"];
                    mState["Imaging"] = eTrackingState::NULL_STATE;
                }

            }
        }
        */
        //   else
        //    if (bOK) {
        //        pnext_track_state = state_options[cam_cur]["NORMAL"];
        //        next_state = eTrackingState::NORMAL;
        //    } else {
        //  else
        if (!bOK){
            if (cam_cur == "SLAM") {
                pnext_track_state = state_options[cam_cur]["REINITIALIZE"];
                next_state = eTrackingState::REINITIALIZE;
                std::cout << "SLAM CAMERA LOST TRACKING, TRYING TO REINITIALIZE: "<< std::endl;

                if(state.find("Imaging") != state.end()){ //if there's an imaging camera set it to NULL state b/c we can't track it if SLAM tracking is lost
                    state["Imaging"]  = state_options["Imaging"]["NULL"];
                    mState["Imaging"] = eTrackingState::NULL_STATE;
                }

            } else {
                next_state = eTrackingState::INITIALIZATION;
                pnext_track_state = state_options[cam_cur]["INITIALIZATION"];

            }
        }
    }
    else if(mState[cam_cur] == eTrackingState::RELOCALIZATION){
        if(bOK){
            //std::cout << "transitioning from relocation to normal tracking state" << std::endl;
            pnext_track_state = state_options[cam_cur]["NORMAL"];
            next_state = eTrackingState::NORMAL;

            if(cam_cur == "SLAM" && state.find("Imaging") != state.end()){ //if the SLAM camera was localized and there's an imaging camera, can allow imaging to start operating again
                mState["Imaging"] = eTrackingState::INITIALIZATION;
                state["Imaging"] = state_options["Imaging"]["INITIALIZATION"];
            }
        }
    } else if(mState[cam_cur] == eTrackingState::REINITIALIZE) {
        if (bOK) {
            //  HandlePostInit(newKFs.back(), maps[cam_cur].get(),  cam_cur);
            next_state = eTrackingState::POSTINIT;
            pnext_track_state = state_options[cam_cur]["POSTINIT"];
            recent_init[cam_cur] = 5;
            state_options[cam_cur]["REINITIALIZE"]->clear();

            if (cam_cur == "SLAM" && state.find("Imaging") != state.end()) { //if the SLAM camera was localized and there's an imaging camera, can allow imaging to start operating again
                mState["Imaging"] = eTrackingState::INITIALIZATION;
                state["Imaging"] = state_options["Imaging"]["INITIALIZATION"];
            }
        }
    }
    mState[cam_cur] = next_state;
    state[cam_cur] = pnext_track_state;
}




}