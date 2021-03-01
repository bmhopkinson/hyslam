#ifndef ORBSLAM_DATASTRUCTS_H_
#define ORBSLAM_DATASTRUCTS_H_

#include <list>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <FeatureExtractor.h>

namespace HYSLAM
{

struct Imgdata
{
  std::string name;
  std::string camera;
  double time_stamp;

  Imgdata(std::string name_, std::string camera_, double time_stamp_);
  Imgdata(){};
};



struct optInfo{
      double Info_IMU;
      double Info_Depth;
      double Info_GPS;
      double Info_TrajTime = 1;
      double Info_TrajTimeSE3 = 1;
      double Info_ImagingTcam = 1;
      bool realtime; // program is being run realtime
      int GBAinterval; //run global BA after this many local BAs
      int GBAtype = 0; //1 = periodic GBA, 2 = Loop Closure GBA
};

struct ORBExtractorParams{
  ORBExtractorParams(){};
  ORBExtractorParams(ORBextractor* extractor);
  void setParams(ORBextractor* extractor);
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvInvScaleFactors;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

};



} //end ORB_SLAM2 namespace
#endif
