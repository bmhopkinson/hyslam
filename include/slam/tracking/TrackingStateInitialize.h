#ifndef TRACKINGSTATEINITIALIZE_H_
#define TRACKINGSTATEINITIALIZE_H_

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <src/main/ORBSLAM_datastructs.h>
#include <Camera.h>
#include <Initializer.h>
#include <vector>
#include <opencv2/core/core.hpp>

namespace HYSLAM {
    class TrackingStateInitialize : public TrackingState {
    public:
        TrackingStateInitialize(optInfo optimizer_info_, Camera camera_, InitializerData &init_data_, StateInitializeParameters params_,  std::ofstream &log);
        bool initialPoseEstimation( Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map< std::string, std::unique_ptr<Trajectory> > &trajectories); //signature mimics TrackingStrategy
        bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map< std::string, std::unique_ptr<Trajectory> > &trajectories);
        void clear();

    private:
        bool needNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force);
        KeyFrame* createNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper);
        int HandlePostMonoInitSLAM(KeyFrame* pKFini, KeyFrame* pKFcur, Map* pMap, Trajectory* trajectory);
        std::unique_ptr<Initializer> initializer;
        optInfo optimizer_info;
        StateInitializeParameters params;
        Camera camera;
        InitializerData* init_data;
        bool success;
        std::vector<KeyFrame*> KFnew;

    };

}

#endif