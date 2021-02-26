#include <LocalBundleAdjustmentJob.h>

namespace HYSLAM{
LocalBundleAdjustmentJob::LocalBundleAdjustmentJob(KeyFrame *pKF_, Map *pMap_, g2o::Trajectory &trajectory_,optInfo optParams_, std::ofstream &log_) :
pKF(pKF_), pMap(pMap_)
{
    log = &log_;
    local_bundle_adjustment = std::make_unique<LocalBundleAdjustment>(pKF, &abort_requested, pMap, trajectory_, optParams_);
}

void LocalBundleAdjustmentJob::run(){
    local_bundle_adjustment->Run();
    has_finished = true;
}

}//end namespace