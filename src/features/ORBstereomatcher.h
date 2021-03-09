#ifndef ORBSTEREOMATCHER_H_
#define ORBSTEREOMATCHER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <FeatureExtractor.h>
#include <ORBExtractorParams.h>
#include <ORBViews.h>
#include <Camera.h>

namespace HYSLAM{

class ORBstereomatcher{
public:
  ORBstereomatcher(FeatureExtractor* mpORBextractorLeft_, FeatureExtractor* mpORBextractorRight_, std::vector<cv::KeyPoint> mvKeys_,
                   std::vector<cv::KeyPoint> mvKeysRight_ , cv::Mat mDescriptors_, cv::Mat mDescriptorsRight_, Camera cam_data, ORBExtractorParams orb_params_);
  ORBstereomatcher(FeatureExtractor* mpORBextractorLeft_, FeatureExtractor* mpORBextractorRight_, ORBViews views, Camera cam_data);
  void computeStereoMatches();
  void getData(std::vector<float> &mvuRight_, std::vector<float> &mvDepth_);
  void getData(ORBViews &views);

private:
  FeatureExtractor* mpORBextractorLeft;
  FeatureExtractor* mpORBextractorRight;
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  cv::Mat mDescriptors, mDescriptorsRight;

  ORBExtractorParams orb_params;

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
