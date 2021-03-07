#ifndef ORBSLAM_DATASTRUCTS_H_
#define ORBSLAM_DATASTRUCTS_H_

#include <list>
#include <vector>
#include <string>
#include <mutex>
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

struct ThreadStatus{

    void clearPostStop();

    bool isStopped() const;
    bool isStopRequested() const;
    bool isStoppable() const;
    bool isRelease() const;
    bool isFinished() const;
    bool isFinishRequested() const;
    bool isAcceptingInput() const;
    bool is_stopped = false;

    void setIsStopped(bool isStopped);
    void setStopRequested(bool stopRequested);
    void setStoppable(bool stoppable);
    void setRelease(bool release);
    void setIsFinished(bool isFinished);
    void setFinishRequested(bool finishRequested);
    void setAcceptingInput(bool acceptingInput);

    bool stop_requested = false;
    bool stoppable = true;
    bool release = false;
    bool is_finished = false;
    bool finish_requested = false;
    bool accepting_input = true;

    mutable std::mutex mutex_ts;
};

struct MainThreadsStatus{
    ThreadStatus tracking;
    ThreadStatus mapping;

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
  ORBExtractorParams(FeatureExtractor* extractor);
  void setParams(FeatureExtractor* extractor);
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
