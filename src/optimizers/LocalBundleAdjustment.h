#ifndef LOCAL_BUNDLE_ADJUSTMENT_H_
#define LOCAL_BUNDLE_ADJUSTMENT_H_

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
  list<KeyFrame*> FindLocalKFs(KeyFrame *pCentralKF);  //should probably convert these returned lists to vectors for better correspondence with globalBa and i don't see any need for them to be lists
  list<MapPoint*>  FindLocalMapPoints(const list<KeyFrame*> &lLocalKeyFrames, KeyFrame *pCentralKF);
  list<KeyFrame*> FindFixedKFs( const list<MapPoint*> lLocalMapPoints, KeyFrame *pCentralKF);

  void ClearLBAFlags(const list<KeyFrame*> &lLocalKeyFrames, const list<KeyFrame*> &lFixedKeyFrames, const list<MapPoint*> lLocalMapPoints);

};
} // namespace ORB_SLAM
#endif
