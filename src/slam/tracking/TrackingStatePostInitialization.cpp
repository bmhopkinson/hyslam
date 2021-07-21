//
// Created by cv-bhlab on 7/20/21.
//

#include "TrackingStatePostInitialization.h"

namespace HYSLAM{

TrackingStatePostInitialization::TrackingStatePostInitialization(optInfo optimizer_info_,
                                                                 StateNormalParameters params_, std::ofstream &log,
                                                                 MainThreadsStatus *thread_status_,
                                                                 FeatureFactory *factory) : TrackingStateNormal(
        optimizer_info_, params_, log, thread_status_, factory)  {

}

bool TrackingStatePostInitialization::needNewKeyFrame(Frame &current_frame, Map *pMap, unsigned int last_keyframe_id,
                                                 bool force) {
    //std::cout << "forcing new keyframe due to recent initialization" <<std::endl;
    return true;
}
} //end namespace