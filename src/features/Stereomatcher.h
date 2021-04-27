#ifndef ORBSTEREOMATCHER_H_
#define ORBSTEREOMATCHER_H_

/*
 * class used to match features between stereo images producing single "stereo features"
 * images must be stereo-rectified (and undistortred). creates a lookup table of potential matches in right image by row of left image.
 * then searches for stereomatches for each feature in the left image.
 * takes candidates from right image from look up table and then applies criteria to potential matches:
 *  1. best feature descriptor match and best descriptor distance is less than a threshold (i.e. good match)
 *  2. disparity within range
 *  3. similar octave (e.g. size) THIS NEEDS TO BE UPDATED - NOW EXPLICTLY USING FEATURE SIZE ELSEWHERE
 */

#include <vector>
#include <opencv2/opencv.hpp>
#include <FeatureExtractor.h>
#include <FeatureMatcher.h>
#include <FeatureExtractorSettings.h>
#include <FeatureViews.h>
#include <DescriptorDistance.h>
#include <Camera.h>

namespace HYSLAM{

class Stereomatcher{
public:
  Stereomatcher(FeatureViews views, Camera cam_data, FeatureMatcherSettings settings);
  void computeStereoMatches();
  void getData(std::vector<float> &mvuRight_, std::vector<float> &mvDepth_);
  void getData(FeatureViews &views);

private:
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<FeatureDescriptor> mDescriptors, mDescriptorsRight;

  FeatureExtractorSettings orb_params;
  Camera camera;

  //thresholds
  float TH_HIGH;
  float TH_LOW;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  int N; //number of left keypoints
  float mb; // stereo baseline in meters
  float mbf; //stereo baseline times focal length
};

}//close namespace
#endif
