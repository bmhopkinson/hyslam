//
// Created by cv-bhlab on 3/3/21.
//

#ifndef HYSLAM_IMAGEPROCESSING_H
#define HYSLAM_IMAGEPROCESSING_H

#include <FeatureExtractor.h>
#include <FeatureFactory.h>
#include <DescriptorDistance.h>
#include <Tracking_datastructs.h>
#include <SensorData.h>
#include <InterThread.h>
#include <ThreadSafeQueue.h>
#include <ORBSLAM_datastructs.h>
#include <ORBExtractorParams.h>
#include <Camera.h>

#include <opencv2/core/core.hpp>
#include <string>
#include <map>

namespace HYSLAM {

class ImageProcessing {
public:
    ImageProcessing(FeatureFactory* factory, const std::string &strSettingPath, std::map<std::string, Camera> cam_data_);
    void ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state);
    void ProcessStereoImage(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data, eTrackingState tracking_state);
    void setOutputQueue(ThreadSafeQueue<ImageFeatureData>* output_queue_){output_queue = output_queue_;}

private:
    void LoadSettings(std::string settings_path, ORBextractorSettings &ORBext_settings);
    cv::Mat PreProcessImg(cv::Mat &img, bool mbRGB, float fscale);
    ORBExtractorParams setORBExtractorParams(FeatureExtractor* extractor);

    ThreadSafeQueue<ImageFeatureData>* output_queue;

    FeatureExtractor* mpORBextractorLeft, *mpORBextractorRight;
    FeatureExtractor* mpIniORBextractor;
   // FeatureExtractor* SURFextractor;
    std::shared_ptr<DescriptorDistance> dist_func;

    std::string cam_cur;  //current camera
    std::map<std::string, Camera> cam_data;

    // Current Image(s)
    cv::Mat mImGray;

};
}//end namespace

#endif //HYSLAM_IMAGEPROCESSING_H
