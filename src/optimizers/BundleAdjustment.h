#ifndef BUNDLE_ADJUSTMENT_H_
#define BUNDLE_ADJUSTMENT_H_

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/Trajectory_g2o.h"
#include <unordered_set>
#include <map>

namespace ORB_SLAM2
{

struct OutlierMono {
  g2o::EdgeSE3ProjectXYZ* e;
  KeyFrame * pKF;
  MapPoint* pMP;
};

struct OutlierStereo {
  g2o::EdgeStereoSE3ProjectXYZ* e;
  KeyFrame * pKF;
  MapPoint* pMP;
};

class BundleAdjustment{

public:
  BundleAdjustment(Map* pMap_, g2o::Trajectory &trajectory_, optInfo optParams_);
//  BundleAdjustment(Map* pMap_, optInfo optParams_);
  virtual ~BundleAdjustment();
  virtual void Run() = 0;
  void setOptimizer(g2o::SparseOptimizer* optimizer_) { optimizer = optimizer_; };

  void SetKeyFrameVertices(const list<KeyFrame*> &lKeyFrames, bool fixed);
  void SetIMUEdges( const list<KeyFrame*> &lKeyFrames );
  void SetDepthEdges( const list<KeyFrame*> &lKeyFrames );
  void SetGPSEdges( const list<KeyFrame*> &lKeyFrames );
  void SetMapPointVerticesEdges( const list<MapPoint*> lMapPoints, bool trackEdges, bool bRobust );
  void SetImagingVertices(const list<KeyFrame*> &lKeyFrames);
  void SetImagingEdges(const list<KeyFrame*> &lKeyFrames);
  vector<OutlierMono>   FindOutliersMono(const double thresh);
  vector<OutlierStereo>  FindOutliersStereo(const double thresh);

protected:
  Map* pMap;
  g2o::Trajectory trajectory;
  optInfo optParams;
  g2o::SparseOptimizer* optimizer;

  //vertex id indices
  unsigned long maxKFid = 0;
  int vertex_id = 0;
  std::map<std::string, int> vertex_map;

  vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vector<KeyFrame*> vpEdgeKFMono;
  vector<MapPoint*> vpMapPointEdgeMono;
  vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vector<KeyFrame*> vpEdgeKFStereo;
  vector<MapPoint*> vpMapPointEdgeStereo;
  unordered_set<std::string> sExcludedMPs;

  bool CheckForImagingCamera(const std::list<KeyFrame*> &lKeyFrames);


};
}// end ORBSLAM2 namespace
#endif
