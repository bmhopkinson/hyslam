#ifndef IMAGING_BUNDLE_ADJUSTMENT_H_
#define IMAGING_BUNDLE_ADJUSTMENT_H_

/*
 * class that implements optimzation of imaging camera positions subject to constraints from trajectory of SLAM camera
 * and landmark observations. makes heavy use of BundleAdjustment base class.
 *
 *
 * KeyData:
 * pMap - map for "Imaging" camera
 * img_trajectory - trajectory obtained for imaging camera based solely on visual data
 * segments - represent continuously tracked fragments of overall imaging camera trajectory - frames within a segment have
 * image-based constraints between them
 *
 * Key Functionality:
 *  FindTrackedSegments() - works through img_trajectory identifying continuously tracked sections and placing all keyframe that
 *  are reference keyframes for a frame within each section into a separate segment. each keyframe will at minimum be the reference
 *  keyframe for the frame it was derived from.
 *
 *  AssignStrandedKeyFrames() - attempts to place keyframes that were not placed in a segment into segments based on parent and child
 *  keyframes in spanning tree. keyframes that cannot be assigned to a segment are currently deleted (SetBad). eventually would like to
 *  try SfM approaches or other strategies for placing these keyframes.
 *
 *  DetermineSimilarityTransforms() - for each segment, determines a similarity tranform that minimizes the distance between keyframe positions in the segment
 *  and the expected positions based on position of the SLAM camera at the time the imaging camera image was taken (and rigid transform between SLAM and imaging camera)
 *  uses Horn algorithm to compute tranform. a deficiency of this approach is that many of the segments are essentially linear and so rotation information is absent.
 *  this is corrected later by RotatePosestoAlign().
 *
 *  ApplySimilarityTransforms() - applies similarity transforms computed for each segment in the previous step to keyframes and mappoints
 *  associated with each segment (mappoints are not strictly associated to a single segment so need to ensure tranform is only applied once).
 *
 *  RotatePosestoAlign() - since segments are often linear the Horn transform applied in ApplySimilarityTransform will not properly align
 *  the orientation of the keyframes with that implied by SLAM camera + SLAM->Imaging rigid transform. this step corrects that by determining
 *  a rotation for each segment that best aligns poses of imaging keyframes with expected position (by SLAM camera + SLAM->Imaging rigid transform).
 *  uses OptHelper function PoseAlignmentTransform to calculate rotation, which is done by augmenting camera centers with points that represent
 *  orientation axes and then using Horn method to calculate a transform.
 *
 *  FindAdditionalMapPointMatches() - for each keyframe, finds all landmarks in the viewing frustum (even in different segments) and attempts to
 *  Fuse (FeatureMatcher::Fuse()) all visible landmarks into the frame. this will create cross-segment matches critical for global consistency
 *  of the final map.
 *
 */

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

class ImagingBundleAdjustment : public BundleAdjustment{

    class SubmapData{
    public:
        cv::Mat Tsim; //similarity transform to align segment with slam poses, computed by Horn 1987 currently
        cv::Mat Rsim; // rotation matrix associated with the above similarity
        cv::Mat Talign; //Transform to align pose vectors - Tsim is often not sufficient b/c segments are often nearly linear or planar
    };
public:
  ImagingBundleAdjustment(Map* pMap_,  Trajectory* img_trajectory_, g2o::Trajectory &slam_trajectory_, FeatureFactory* factory, optInfo optParams_);
  void Run();

private:
  Trajectory* img_trajectory;
  const double chi2_thresh_mono = 5.991;   //outlier thresholds
  const double chi2_thresh_stereo = 7.815;
  FeatureFactory* feature_factory;

  std::list<KeyFrame*> KFs_to_optimize;
  std::list<MapPoint*> mpts_to_optimize;
  std::map<std::shared_ptr<Map>, SubmapData> submap_data;

  void DetermineSimilarityTransforms();
  void ApplySimilarityTransforms();
  void RotatePosestoAlign();
  void FindAdditionalMapPointMatches();
  g2o::SparseOptimizer optimizer; //should probably just create this in constructor and std::move to base BundleAdjustment


};
} //namespace HYSLAM
#endif
