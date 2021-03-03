#ifndef ORBVIEWS_H_
#define ORBVIEWS_H_

#include <src/main/ORBSLAM_datastructs.h>
#include <opencv2/opencv.hpp>

#include <vector>

namespace HYSLAM{

class ORBViews{
//ORB keypoints in a frame
public:
  ORBViews(){};
  ORBViews(const ORBViews &views);
  ORBViews& operator=(const ORBViews&);
  ORBViews(std::vector<cv::KeyPoint> mvKeys_, cv::Mat mDescriptors_, ORBExtractorParams orb_params_ );  // mono constructor
  ORBViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,
            cv::Mat mDescriptors_, cv::Mat mDescriptorsRight_, ORBExtractorParams orb_params_);  //stereo constructor - partial data
  ORBViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,  std::vector<float> mvuRight_,  std::vector<float> mvDepth_,
            cv::Mat mDescriptors_, cv::Mat mDescriptorsRight_, ORBExtractorParams orb_params_);  //stereo constructor - full data

  //getters
  bool empty() const {return is_empty;}
  bool isStereo() const {return is_stereo;};
  int numViews() const {return N;}
  cv::KeyPoint keypt(int i ) const {return mvKeys[i];  }
  cv::KeyPoint keyptR(int i ) const;
  cv::Mat descriptor(int i ) const {return mDescriptors.row(i).clone();  }
  cv::Mat descriptorR(int i ) const;
  float uR(int i) const;
  float depth(int i) const;
  ORBExtractorParams orbParams() const { return orb_params; }
  
  // more getters
  ORBExtractorParams getOrbParams()const { return orb_params; }
  std::vector<cv::KeyPoint> getKeys() const {return mvKeys; }
  std::vector<cv::KeyPoint> getKeysR() const {return mvKeysRight; }
  std::vector<float> getuRs() const {return mvuRight; }
  std::vector<float> getDepths() const {return mvDepth; }
  cv::Mat getDescriptors() const { return mDescriptors.clone(); }
  cv::Mat getDescriptorsR() const { return mDescriptorsRight.clone(); }
  
  //setters
  void setOrbParams(ORBExtractorParams params ) {  orb_params  = params; }
  void setKeys(std::vector<cv::KeyPoint> keypts) { mvKeys = keypts; }
  void setKeysR(std::vector<cv::KeyPoint> keyptsR) { mvKeysRight = keyptsR; }
  void setuRs(std::vector<float> uRs ) { mvuRight = uRs; }
  void setDepths(std::vector<float> depths)  {mvDepth = depths; }
  void setDescriptors(cv::Mat descriptors)  {  mDescriptors = descriptors.clone(); }
  void setDescriptorsR(cv::Mat descriptorsR) { mDescriptorsRight = descriptorsR.clone();}

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

//ORB descriptors
  cv::Mat mDescriptors;
  cv::Mat mDescriptorsRight;

  // Scale pyramid info.
 ORBExtractorParams orb_params;

};
}//end namespace
#endif
