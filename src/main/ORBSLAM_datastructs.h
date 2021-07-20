#ifndef ORBSLAM_DATASTRUCTS_H_
#define ORBSLAM_DATASTRUCTS_H_

/*
 * LEGACY - set of disjointed data structures - need to revise
 * Imgdata holds basic data about an image
 *
 * optInfo holds parameters for optimization/ bundle adjustment
 */

#include <FeatureExtractor.h>
#include <FeatureViews.h>
#include <SensorData.h>

#include <list>
#include <vector>
#include <string>
#include <mutex>
#include <opencv2/opencv.hpp>

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
      double Info_submap_tiepoint;
      bool realtime; // program is being run realtime
      int GBAinterval; //run global BA after this many local BAs
      int GBAtype = 0; //1 = periodic GBA, 2 = Loop Closure GBA
};



} //end ORB_SLAM2 namespace
#endif
