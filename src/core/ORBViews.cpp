#include <ORBViews.h>

namespace ORB_SLAM2{
//copy constructor
ORBViews::ORBViews(const ORBViews &views){
  is_stereo = views.is_stereo;
  is_empty = views.is_empty;
  N = views.N;
  mvKeys = views.mvKeys;
  mvKeysRight= views.mvKeysRight;
  mvKeysUn = views.mvKeysUn;
  mvuRight = views.mvuRight;
  mvDepth = views.mvDepth;
  mDescriptors = views.mDescriptors.clone(); //deep copy, why we need explicit copy constructor
  mDescriptorsRight = views.mDescriptorsRight.clone();
  orb_params = views.orb_params;
}

//assignment operator
ORBViews& ORBViews::operator=(const ORBViews& views){
    if(this == &views)
        return *this;

    is_stereo = views.is_stereo;
    is_empty = views.is_empty;
    N = views.N;
    mvKeys = views.mvKeys;
    mvKeysRight= views.mvKeysRight;
    mvKeysUn = views.mvKeysUn;
    mvuRight = views.mvuRight;
    mvDepth = views.mvDepth;
    mDescriptors = views.mDescriptors.clone(); //deep copy, why we need explicit assignment constructor
    mDescriptorsRight = views.mDescriptorsRight.clone();
    orb_params = views.orb_params;

    return *this;
}

// mono constructor
ORBViews::ORBViews(std::vector<cv::KeyPoint> mvKeys_,cv::Mat  mDescriptors_, ORBExtractorParams orb_params_ ){
  is_stereo = false;
  N = mvKeys_.size();
  if(N>0){
    is_empty = false;
  }

  mvKeys = mvKeys_;
  mvKeysUn = mvKeys_;
  mDescriptors = mDescriptors_;
  orb_params = orb_params_;

}

//stereo constructor- partial data
ORBViews::ORBViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,
            cv::Mat mDescriptors_, cv::Mat mDescriptorsRight_, ORBExtractorParams orb_params_){
  is_stereo = true;
  N = mvKeys_.size();
  if(N>0){
    is_empty = false;
  }

  mvKeys = mvKeys_;
  mvKeysRight = mvKeysRight_;
  mDescriptors = mDescriptors_;
  mDescriptorsRight = mDescriptorsRight_;
  orb_params = orb_params_;
}  

//stereo constructor- full data
ORBViews::ORBViews(std::vector<cv::KeyPoint> mvKeys_, std::vector<cv::KeyPoint> mvKeysRight_,  std::vector<float> mvuRight_,  std::vector<float> mvDepth_,
            cv::Mat mDescriptors_, cv::Mat mDescriptorsRight_, ORBExtractorParams orb_params_)
{
  is_stereo = true;
  N = mvKeys_.size();
  if(N>0){
    is_empty = false;
  }

  mvKeys = mvKeys_;
  mvKeysRight = mvKeysRight_;
  mvKeysUn = mvKeys_;
  mvuRight = mvuRight_;
  mvDepth = mvDepth_;
  mDescriptors = mDescriptors_;
  mDescriptorsRight = mDescriptorsRight_;
  orb_params = orb_params_;

}

cv::KeyPoint ORBViews::keyptR(int i ) const {
if(is_stereo){
    return mvKeysRight[i];  }
else {
   return cv::KeyPoint(); //empty Keypoint
}
}

cv::Mat ORBViews::descriptorR(int i ) const {
if(is_stereo){
    return mDescriptorsRight.row(i).clone();  }
else {
   return cv::Mat(); //empty mat
}
}

float ORBViews::uR(int i) const {
if(is_stereo){
    return mvuRight[i]; }
else {
   return -1.0;
}
}

float ORBViews::depth(int i) const{
if(is_stereo){
    return mvDepth[i]; }
else {
   return -1.0;
}
}

}//close namespace
