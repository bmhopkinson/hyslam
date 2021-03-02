#ifndef GLOBAL_BUNDLE_ADJUSTMENT_H_
#define GLOBAL_BUNDLE_ADJUSTMENT_H_

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "BundleAdjustment.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/Trajectory_g2o.h"

#include <vector>
#include <list>

namespace HYSLAM
{
class GlobalBundleAdjustment : public BundleAdjustment {

public:
  GlobalBundleAdjustment(const std::vector<KeyFrame*> &vpKFfix_, const unsigned long nLoopKF_, int nIterations_, const bool bRobust_, bool* pbStopFlag_, Map* pMap_,  g2o::Trajectory &trajectory_, optInfo optParams_);

  void Run();

private:
  //variables
  std::vector<KeyFrame*> vpKFfix;
  unsigned long nLoopKF;
  int nIterations = 10;
  bool bRobust;
  bool* pbStopFlag = nullptr;
  g2o::SparseOptimizer optimizer;

  static int ncalls;

  //helper functions
  void RecoverOptimizedKeyFrames( std::list<KeyFrame*> lKeyFrames);
  void RecoverOptimizedMapPoints(  std::list<MapPoint*>  lMapPoints);
};

} //end ORB_SLAM2 namespace

#endif
