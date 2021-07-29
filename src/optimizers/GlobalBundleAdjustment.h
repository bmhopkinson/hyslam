#ifndef GLOBAL_BUNDLE_ADJUSTMENT_H_
#define GLOBAL_BUNDLE_ADJUSTMENT_H_

/*
 * class that implements a global bundle adjustment. relies heavily on BundleAdjustment base class
 * does global bundle adjustment of pMap, keeping keyframes in vpKFfix constant
 * can be used either as periodic global bundle adjustment in which case keyframe poses and landmark positions are
 * automatically updated post optimization in RecoverOptimizedKeyFrame and RecoverOptimizedMapPoints, or can be called
 * during loop closing in which case updates are deferred in case new keyframes/landmarks have been added to the map in the mean time
 * (for loop closing updated data is stored in KeyFrame and MapPoint objects.)
 * options - including loop closing vs periodic mode are specified in optParams
 *
 * makes use of constraints from:
 *  LandMark observations
 *  IMU data
 *  GPS data
 *  Depth data
 *  when those data sources are available
 *
 *  usage:
 *  simply construct the object and then Run(). optimization can take along time so best to run in a separate thread.
 *  remember that for loop closing the updated pose/positions must be manually applied later.
 *
 */

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
  std::list<Tse3Parent>  FindSubmapTiepoints(const std::list<KeyFrame*> &KeyFrames);
  std::list<KeyFrame*>  FindAdditionalParentSubmapKFs(const std::list<Tse3Parent> &submap_tiepoints, const std::set<KeyFrame*> &currentKFs );
  void RecoverOptimizedKeyFrames( std::list<KeyFrame*> lKeyFrames);
  void RecoverOptimizedMapPoints(  std::list<MapPoint*>  lMapPoints);

};

} //end ORB_SLAM2 namespace

#endif
