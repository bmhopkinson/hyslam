#ifndef HYSLAM_FEATUREVIEWS_H_
#define HYSLAM_FEATUREVIEWS_H_

/*
 * class representing features + descriptors in a single image or keyframe - mostly a data structure, little functionality
 * core data is:
 *  keypoint (cv::Keypoint) data stored in mvKeys and for stereo frames mvKeysRight
 *  associated keypoint descriptors (mDescriptors and for stereo mDescriptorsRight
 *  stereo info: mvuRight (column coordinate for stereomatched keypoint, -1 if not stereo), mvDepth (distance along principal axis) - NOTE: not all keypoints in a stereo-image have stereomatches
 *  all are organized as vectors of same length, indexed by keypoint number
 */

#include <FeatureExtractorSettings.h>
#include <opencv2/opencv.hpp>
#include <FeatureDescriptor.h>
#include <vector>

namespace HYSLAM{

class FeatureViews{
//ORB keypoints in a frame
public:
  FeatureViews(){};
  FeatureViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<FeatureDescriptor> mDescriptors_, FeatureExtractorSettings orb_params_ );  // mono constructor
  FeatureViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,
               std::vector<FeatureDescriptor> mDescriptors_, std::vector<FeatureDescriptor> mDescriptorsRight_, FeatureExtractorSettings orb_params_);  //stereo constructor - partial data
  FeatureViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,  std::vector<float> mvuRight_,  std::vector<float> mvDepth_,
               std::vector<FeatureDescriptor> mDescriptors_, std::vector<FeatureDescriptor> mDescriptorsRight_, FeatureExtractorSettings orb_params_);  //stereo constructor - full data

  //getters
  bool empty() const {return is_empty;}
  bool isStereo() const {return is_stereo;};
  int numViews() const {return N;}
  cv::KeyPoint keypt(int i ) const {return mvKeys[i];  }
  cv::KeyPoint keyptR(int i ) const;
  const FeatureDescriptor& descriptor(int i ) const {return mDescriptors[i];  }
  float uR(int i) const;
  float depth(int i) const;
  FeatureExtractorSettings orbParams() const { return orb_params; }
  
  // more getters
  FeatureExtractorSettings getOrbParams()const { return orb_params; }
  std::vector<cv::KeyPoint> getKeys() const {return mvKeys; }
  std::vector<cv::KeyPoint> getKeysR() const {return mvKeysRight; }
  std::vector<float> getuRs() const {return mvuRight; }
  std::vector<float> getDepths() const {return mvDepth; }
  std::vector<FeatureDescriptor> getDescriptors() const { return mDescriptors; }
  std::vector<FeatureDescriptor> getDescriptorsR() const { return mDescriptorsRight; }
  
  //setters
  void setOrbParams(FeatureExtractorSettings params ) {  orb_params  = params; }
  void setKeys(std::vector<cv::KeyPoint> keypts) { mvKeys = keypts; }
  void setKeysR(std::vector<cv::KeyPoint> keyptsR) { mvKeysRight = keyptsR; }
  void setuRs(std::vector<float> uRs ) { mvuRight = uRs; }
  void setDepths(std::vector<float> depths)  {mvDepth = depths; }

private:

  bool is_stereo = false;
  bool is_empty = true;
  int N = 0; //number of keypoints

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
  // In the stereo case, mvKeysUn is redundant as images must be rectified.
  // In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

//Feature descriptors
    std::vector<FeatureDescriptor> mDescriptors;
    std::vector<FeatureDescriptor> mDescriptorsRight;

  // Scale pyramid info.
 FeatureExtractorSettings orb_params;

};
}//end namespace
#endif
