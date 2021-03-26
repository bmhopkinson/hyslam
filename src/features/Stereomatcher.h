#ifndef ORBSTEREOMATCHER_H_
#define ORBSTEREOMATCHER_H_

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
