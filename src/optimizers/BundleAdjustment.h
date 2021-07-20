#ifndef BUNDLE_ADJUSTMENT_H_
#define BUNDLE_ADJUSTMENT_H_

/*
 * general purpose non linear optimzation (bundle adjustment) class
 * used as a base class by other more specialized bundle adjustment methods
 * g2o is used for non-linear optimization
 * NOTE: this was written up early on in refactoring of ORB SLAM and some design decisions are sub-optimal - for example there are functions
 * specific to imaging bundle adjustment in this general class
 *
 * Important data:
 * pMap: map containting data (KeyFrames, Landmarks, etc) to optimize
 * trajectory: trajectory of localization (SLAM) camera
 * optimizer: g2o graph optimizer - holds vertices, edges, and performs optimizations
 * vertex_map: maps between vertex names (in a standard format) and their integer id in the optimzer - for recovery post optimization
 *
 * Key Functions:
 * SetKeyFrameVertices(const std::list<KeyFrame*> &lKeyFrames, bool fixed) - adds keyframes in the list to the optimizer. if fixed = true optimizer
 * is not permitted to adjust their poses. otherwise sets initial position estimate to current pose.
 *
 * SetIMUEdges( const std::list<KeyFrame*> &lKeyFrames ) - creates unary edges (priors) between lKeyFrames, which must already be
 * in the optimizer, and IMU estimate of orientation (in quaternion form) if IMU data is available. sets error estimate based on optParams.
 *
 * SetDepthEdges( const std::list<KeyFrame*> &lKeyFrames ) - creates unary edges (priors) between lKeyFrames, which must already be
 * in the optimizer, and depth estimate if it is available. sets error estimate based on optParams.
 *
 * SetGPSEdges( const std::list<KeyFrame*> &lKeyFrames )- creates unary edges (priors) between lKeyFrames, which must already be
 * in the optimizer, and GPS position estimate if it is available. because the SLAM positions are in an arbitrary reference frame
 * while the GPS data is an absolute (and invariably different) reference frame, a GPS to SLAM coordinate transform (currently a similarity but could fix scale)
 * is calculated using Horn's method and applied to the GPS data. requires at least 4 GPS measurements. sets error estimate based on optParams.
 *
 * SetMapPointVerticesEdges( const std::list<MapPoint*> lMapPoints, bool trackEdges, bool bRobust ) - sets vertices for landmarks in lMapPoints and binary edges between
 * those landmarks and keyframes based on observations (which are known by landmarks). determines whether each landmark observation is a monocular or stereo observation
 * and creates appropriate edge. if trackEdges = true will record edges in vpEdgeMono, ..., vpMapPointEdgeStereo; typically this is used for outlier exclusion after initial
 * bundle adjustment.
 *
 * SetImagingVertices(const std::list<KeyFrame*> &lKeyFrames) - sets vertices for lKeyFrames from an imaging camera. the position is constrained in part by
 * the time was image was taken and what the implies about the position of the SLAM camera at that time. consequently, initial estimate of time at which image was taken
 * is set as the image time stamp. Imaging bundle adjustment also requires a rigid transform bewteen the SLAM camera and Imaging camera to be estimated. a vertex representing this
 * transform is created.
 *
 * SetImagingEdges(const std::list<KeyFrame*> &lKeyFrames) - sets 3 part edges for each imaging keyframe (lKeyFrames) representing constraints
 * on pose of keyframe from 1: time at which image was taken and hence implied position of SLAM camera, 2: rigid transform between SLAM and Imaging camera
 * vertices involved represent 1: time at which image was taken (and hence implied position of SLAM camera), 2: SLAM->imaging Tranform, 3: imaging camera pose
 *
 */

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/Trajectory_g2o.h"
#include <unordered_set>
#include <map>
#include <list>

namespace HYSLAM
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

  void SetKeyFrameVertices(const std::list<KeyFrame*> &lKeyFrames, bool fixed);
  void SetIMUEdges( const std::list<KeyFrame*> &lKeyFrames );
  void SetDepthEdges( const std::list<KeyFrame*> &lKeyFrames );
  void SetGPSEdges( const std::list<KeyFrame*> &lKeyFrames );
  void SetSubMapOriginEdges(const std::list<Tse3Parent> &submap_tiepoints);
  void SetMapPointVerticesEdges( const std::list<MapPoint*> lMapPoints, bool trackEdges, bool bRobust );
  void SetImagingVertices(const std::list<KeyFrame*> &lKeyFrames);
  void SetImagingEdges(const std::list<KeyFrame*> &lKeyFrames);
  std::vector<OutlierMono>   FindOutliersMono(const double thresh);
  std::vector<OutlierStereo>  FindOutliersStereo(const double thresh);

protected:
  Map* pMap;
  g2o::Trajectory trajectory;
  optInfo optParams;
  g2o::SparseOptimizer* optimizer;

  //vertex id indices
  unsigned long maxKFid = 0;
  int vertex_id = 0;
  std::map<std::string, int> vertex_map;

  std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
  std::vector<KeyFrame*> vpEdgeKFMono;
  std::vector<MapPoint*> vpMapPointEdgeMono;
  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  std::vector<KeyFrame*> vpEdgeKFStereo;
  std::vector<MapPoint*> vpMapPointEdgeStereo;
  std::unordered_set<std::string> sExcludedMPs;

  bool CheckForImagingCamera(const std::list<KeyFrame*> &lKeyFrames);


};
}// end ORBSLAM2 namespace
#endif
