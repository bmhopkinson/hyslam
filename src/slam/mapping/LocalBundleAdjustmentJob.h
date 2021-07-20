#ifndef LOCALBUNDLEADJUSTMENTJOB_H_
#define LOCALBUNDLEADJUSTMENTJOB_H_

/*
 * map job that runs a LocalBundleAdjustment centered on pKF_
 *
 * this is just a wrapper to LocalBundleAdjustment, derived from MapJob so it can be handled identically to other map jobs
 *
 */

#include <MapJob.h>
#include <KeyFrame.h>
#include <Map.h>
#include <LocalBundleAdjustment.h>
#include <ORBSLAM_datastructs.h>
#include <g2o/types/sba/Trajectory_g2o.h>

#include <iostream>

namespace HYSLAM{
    class LocalBundleAdjustmentJob : public MapJob {
    public:
        LocalBundleAdjustmentJob(KeyFrame *pKF_, Map *pMap_, g2o::Trajectory &trajectory_,optInfo optParams_, std::ofstream &log_);
        void run();
        std::string name(){return "LocalBundleAdjustmentJob";};

    private:
        KeyFrame* pKF;
        Map* pMap;
        std::ofstream* log;
        std::unique_ptr<LocalBundleAdjustment> local_bundle_adjustment;
    };

}//end namespace
#endif