#ifndef LOCAL_BUNDLE_ADJUSTMENT_H_
#define LOCAL_BUNDLE_ADJUSTMENT_H_

/*
 * class that implements optimization of  camera and landmark positions in a local region around the newest keyframe (pCentralKF)
 * makes heavy use of BundleAdjustment base class. primarily this class finds the local keyframes and landmarks near pCentralKF
 * and then relies on functionality of BundleAdjustment base to do the optimization
 *
 * KeyFunctions:
 * FindLocalKFs(KeyFrame *pCentralKF) - all keyframes connected in the covisibility graph to pCentralKF are considered "local".
 *  the pose of these local keyframes will be modified by optimization
 * FindLocalMapPoints() - collects all landmarks observed in local keyframes as the local mappoints.
 * FindFixedKFs() - peripheral keyframes that observe local landmarks (but are not in the local keyframe list).
 * these keyframes will be included as constraints in the optimization but their positions will be fixed.
 *
 * Run() - finds local keyframes, local landmarks, and fixed keyframes. sets up vertices and edges between keyframes,
 * landmarks, and accessory sensor data (IMU, GPS, depth). runs an initial optimization (currently 5 iterations) and
 * then removes outlier edges, and optimizes again. after successful optimization, updates pose of local keyframe and
 * positions of local landmarks based on optimization.
 *
 *
 */

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "BundleAdjustment.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/Trajectory_g2o.h"

namespace HYSLAM
{
class LocalBundleAdjustment : public BundleAdjustment {



public:
  LocalBundleAdjustment(KeyFrame *pCentralKF_,  bool* pbStopFlag_, Map* pMap_, g2o::Trajectory &trajectory_, optInfo optParams_);
  //LocalBundleAdjustment( KeyFrame *pCentralKF_,  bool* pbStopFlag_, Map* pMap_,  optInfo optParams_);
  void Run();

private:
  //variables
  KeyFrame *pCentralKF;
  g2o::SparseOptimizer optimizer;
  bool* pbStopFlag;

  bool imaging_camera_exists;

  static int ncalls;

  //helper functions
    std::list<KeyFrame*> FindLocalKFs(KeyFrame *pCentralKF);  //should probably convert these returned lists to vectors for better correspondence with globalBa and i don't see any need for them to be lists
    std::list<MapPoint*>  FindLocalMapPoints(const std::list<KeyFrame*> &lLocalKeyFrames, KeyFrame *pCentralKF);
    std::list<KeyFrame*> FindFixedKFs( const std::list<MapPoint*> lLocalMapPoints, KeyFrame *pCentralKF);

  void ClearLBAFlags(const std::list<KeyFrame*> &lLocalKeyFrames, const std::list<KeyFrame*> &lFixedKeyFrames, const std::list<MapPoint*> lLocalMapPoints);

};
} // namespace ORB_SLAM
#endif
