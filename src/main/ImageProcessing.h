//
// Created by cv-bhlab on 3/3/21.
//

#ifndef HYSLAM_IMAGEPROCESSING_H
#define HYSLAM_IMAGEPROCESSING_H

/*
 * class that processing images to extract keypoint features for matching (could also do things like contrast enhancement, etc)
 * after processing image it adds the results (keypoint features, etc) to the output_queue for tracking to work on.
 *
 * key functionality:
 * ImageProcessing(FeatureFactory* factory_, const std::string &strSettingPath, std::map<std::string, Camera> cam_data_)
 *    constructor - provide access to the feature set that will be used through factory_, settings for feature extraction - CAN BE REMOVED, and camera data
 *    gets feature extractors from factory.
 *
 *  cv::Mat PreProcessImg(cv::Mat &img, bool mbRGB, float fscale) - image preprocessing prior to feature extraction.
 *      scales image to requested scale and converts to grayscale if that hasn't already been done.
 *
 *  ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state)
 *      preprocesses and extracts keypoint features from im (from a monocular camera). extraction may depend on tracking_state (currently extract more points during initialization)
 *      after feature extractions, package feature data and accessory data (img_data, sensor_data) into a ImageFeatureData object and put onto
 *      output queue for tracking to handle
 *
 *  ProcessStereoImage(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data, eTrackingState tracking_state);
 *      preprocesses and extracts keypoint features from imRectLeft and imRectRight (stereo rectified images from stereo camera). right image is extracted in a seperate thread.
 *      after feature extraction do stereomatching using StereoMatcher to associate features between left and right image.
 *      extraction may depend on tracking_state (but currently doesn't)
 *      after feature extractions, package feature data and accessory data (img_data, sensor_data) into a ImageFeatureData object and put onto
 *      output queue for tracking to handle
 *
 */

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
    cv::Mat PreProcessImg(cv::Mat &img, bool mbRGB, float fscale);
    FeatureMatcherSettings feature_settings;
    FeatureFactory* factory;

    ThreadSafeQueue<ImageFeatureData>* output_queue;

    FeatureExtractor* extractor_left, *extractor_right;
    FeatureExtractor* extractor_init;

    std::string cam_cur;  //current camera
    std::map<std::string, Camera> cam_data;

    static int n_processed;

    // Current Image(s)
    cv::Mat mImGray;

};
}//end namespace

#endif //HYSLAM_IMAGEPROCESSING_H
