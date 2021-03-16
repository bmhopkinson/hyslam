#ifndef ORBSTEREOMATCHER_H_
#define ORBSTEREOMATCHER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <FeatureExtractor.h>
#include <FeatureMatcher.h>
#include <ORBExtractorParams.h>
#include <FeatureViews.h>
#include <DescriptorDistance.h>
#include <Camera.h>

namespace HYSLAM{

class ORBstereomatcher{
public:
  ORBstereomatcher(FeatureExtractor* mpORBextractorLeft_, FeatureExtractor* mpORBextractorRight_, FeatureViews views,
                   DescriptorDistance* descriptor_distance_, Camera cam_data, FeatureMatcherSettings settings);
  void computeStereoMatches();
  void getData(std::vector<float> &mvuRight_, std::vector<float> &mvDepth_);
  void getData(FeatureViews &views);

private:
  FeatureExtractor* mpORBextractorLeft;
  FeatureExtractor* mpORBextractorRight;
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<FeatureDescriptor> mDescriptors, mDescriptorsRight;
  DescriptorDistance* descriptor_distance;

  ORBExtractorParams orb_params;

  //thresholds
  int TH_HIGH;
  int TH_LOW;

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
