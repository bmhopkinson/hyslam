//
// Created by cv-bhlab on 7/20/21.
//

#ifndef HYSLAM_TRACKINGSTATEPOSTINITIALIZATION_H
#define HYSLAM_TRACKINGSTATEPOSTINITIALIZATION_H

#include <TrackingStateNormal.h>

//same thing as TrackingStateNormal but forces new KeyFrame

namespace HYSLAM {

class TrackingStatePostInitialization : public TrackingStateNormal {
public:
    TrackingStatePostInitialization(optInfo optimizer_info_, StateNormalParameters params_, std::ofstream &log,
                                    MainThreadsStatus *thread_status_, FeatureFactory *factory);

protected:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force);
};
} //end namespace

#endif //HYSLAM_TRACKINGSTATEPOSTINITIALIZATION_H
