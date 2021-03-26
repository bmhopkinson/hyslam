//
// Created by cv-bhlab on 3/3/21.
//

#ifndef HYSLAM_IMAGEPROCESSING_H
#define HYSLAM_IMAGEPROCESSING_H

#include <FeatureExtractor.h>
#include <FeatureFactory.h>
#include <FeatureMatcher.h>
#include <DescriptorDistance.h>
#include <Tracking_datastructs.h>
#include <SensorData.h>
#include <InterThread.h>
#include <ThreadSafeQueue.h>
#include <ORBSLAM_datastructs.h>
#include <FeatureExtractorSettings.h>
#include <Camera.h>
#include <SURFExtractor.h>

#include <opencv2/core/core.hpp>
#include <string>
#include <map>

namespace HYSLAM {

class ImageProcessing {
public:
    ImageProcessing(FeatureFactory* factory_, const std::string &strSettingPath, std::map<std::string, Camera> cam_data_);
    void ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state);
    void ProcessStereoImage(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data, eTrackingState tracking_state);
    void setOutputQueue(ThreadSafeQueue<ImageFeatureData>* output_queue_){output_queue = output_queue_;}

private:
    void LoadSettings(std::string settings_path, FeatureExtractorSettings &feature_extractor_settings, FeatureMatcherSettings &feature_settings);
    cv::Mat PreProcessImg(cv::Mat &img, bool mbRGB, float fscale);
  //  FeatureExtractorSettings setFeatureExtractorSettings(FeatureExtractor* extractor);
    FeatureMatcherSettings feature_settings;
    FeatureFactory* factory;

    ThreadSafeQueue<ImageFeatureData>* output_queue;

    FeatureExtractor* extractor_left, *extractor_right;
    FeatureExtractor* extractor_init;
    SURFExtractor* SURFextractor;
    std::shared_ptr<DescriptorDistance> dist_func;
    std::shared_ptr<DescriptorDistance> dist_func_surf;

    std::string cam_cur;  //current camera
    std::map<std::string, Camera> cam_data;

    static int n_processed;

    // Current Image(s)
    cv::Mat mImGray;

};
}//end namespace

#endif //HYSLAM_IMAGEPROCESSING_H
