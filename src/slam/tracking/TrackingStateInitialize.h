#ifndef TRACKINGSTATEINITIALIZE_H_
#define TRACKINGSTATEINITIALIZE_H_

/*
 * TrackingState for initialization of tracking - makes use of underlying Initializer class 
 *    to do the actual initialization (currently there are monocular and stereo initializers but could be
 *    expanded for other cameras or be made condition dependent).
 *    needs access to FeatureFactory because monocular initialization relies on feature matching. 
 * 
 *  key functions:
 *  initialPoseEstimation() - passes current_frame to initializer and if initializer is succesful
 *   has the initializer create a map and new keyframes (stored in KFnew). handles alignment of imaging camera with SLAM in a opaque manner, which
 *   really should be updated - tests if camera is monocular and not SLAM (i.e. must be imaging) and if so calls initializer->transformMap();
 *   if the camera is a monocular SLAM camera calls a special helper function HandlePostMonoInitSLAM()
 *  
 *  refinePoseEstimate() - dummy function, just returns true.
 * 
 *  needNewKeyFrame() - always returns true b/c we'll always need a new keyframe when a camera is first initialized. 
 *
 *  createNewKeyFrame() - returns keyframes stored in KFnew, updates current_frame whose pose may have been altered upon initialization,
 *    also sets pMap->mvpKeyFrameOrigins to current keyframe - this is an obscure piece of data used in loop closing. 
 *  
 *  HandlePostMonoInitSLAM() - runs global bundle adjustment and scales the map so that the median scene depth is one.  
 * 
 */

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <InterThread.h>
#include <Camera.h>
#include <Initializer.h>
#include <FeatureFactory.h>
#include <vector>
#include <opencv2/core/core.hpp>

namespace HYSLAM {
    class TrackingStateInitialize : public TrackingState {
    public:
        TrackingStateInitialize(optInfo optimizer_info_, Camera camera_, InitializerData &init_data_,
                                StateInitializeParameters params_,  std::ofstream &log, MainThreadsStatus* thread_status_,
                                FeatureFactory* factory);
        bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories); //signature mimics TrackingStrategy
        bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories);
        void clear();

    private:
        bool needNewKeyFrame(Frame &current_frame, Map* pMap,  unsigned int last_keyframe_id, bool force);
        std::vector<KeyFrame*> createNewKeyFrame(Frame &current_frame, Map* pMap);
        int HandlePostMonoInitSLAM(KeyFrame* pKFini, KeyFrame* pKFcur, Map* pMap, Trajectory* trajectory);
        std::unique_ptr<Initializer> initializer;
        optInfo optimizer_info;
        StateInitializeParameters params;
        Camera camera;
        InitializerData* init_data;
        FeatureFactory* feature_factory;
        bool success;
        std::vector<KeyFrame*> KFnew;

    };

}

#endif