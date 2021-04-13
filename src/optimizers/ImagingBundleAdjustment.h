#ifndef IMAGING_BUNDLE_ADJUSTMENT_H_
#define IMAGING_BUNDLE_ADJUSTMENT_H_

#include "BundleAdjustment.h"
#include "Trajectory.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include <FeatureFactory.h>

#include <set>
#include <list>

#include "g2o/core/sparse_optimizer.h"

namespace HYSLAM
{


class Segment {
  public:
    Segment();
    Segment(const Segment &seg);
    Segment& operator=(const Segment &seg);
    std::set<KeyFrame*> key_frames;
    std::set<MapPoint*> map_points;
    cv::Mat Tsim; //similarity transform to align segment with slam poses, computed by Horn 1987 currently
    cv::Mat Rsim; // rotation matrix associated with the above similarity
    cv::Mat Talign; //Transform to align pose vectors - Tsim is often not sufficient b/c segments are often nearly linear or planar
};

typedef std::vector< Segment > TrackedSegments;

class ImagingBundleAdjustment : public BundleAdjustment{

public:
  ImagingBundleAdjustment(Map* pMap_,  Trajectory* img_trajectory_, g2o::Trajectory &slam_trajectory_, FeatureFactory* factory, optInfo optParams_);
  void Run();

private:
  Trajectory* img_trajectory;
  TrackedSegments segments;
  const double chi2_thresh_mono = 5.991;   //outlier thresholds
  const double chi2_thresh_stereo = 7.815;
  FeatureFactory* feature_factory;

  std::list<KeyFrame*> KFs_to_optimize;
  std::list<MapPoint*> mpts_to_optimize;


  void FindTrackedSegments();
  void AssignStrandedKeyFrames();
  void DetermineSimilarityTransforms();
  void ApplySimilarityTransforms();
  void RotatePosestoAlign();
  void FindAdditionalMapPointMatches();
  g2o::SparseOptimizer optimizer; //should probably just create this in constructor and std::move to base BundleAdjustment
};
} //namespace HYSLAM
#endif
