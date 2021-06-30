//
// Created by cv-bhlab on 3/3/21.
//

#include "ImageProcessing.h"
#include <ORBFinder.h>
#include <SURFFinder.h>
#include <FeatureViews.h>
#include <FeatureUtil.h>
#include <Stereomatcher.h>



#include <iostream>
#include <thread>

namespace HYSLAM{
int ImageProcessing::n_processed = 0;

ImageProcessing::ImageProcessing(FeatureFactory* factory_, const std::string &strSettingPath, std::map<std::string, Camera> cam_data_)
 : factory(factory_),cam_data(cam_data_)
{
    // Load camera parameters from settings file
   // FeatureExtractorSettings feature_extractor_settings;
   // LoadSettings(strSettingPath, feature_extractor_settings, feature_settings);


    extractor_left = factory->getExtractor();
    extractor_right = factory->getExtractor();

    FeatureExtractorSettings feature_extractor_settings_init = factory->getFeatureExtractorSettings();
    feature_extractor_settings_init.nFeatures = 3 * feature_extractor_settings_init.nFeatures;
    extractor_init = factory->getExtractor(feature_extractor_settings_init );

}


void ImageProcessing::ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state){
    cam_cur = img_data.camera;
    mImGray = im;
    mImGray = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<FeatureDescriptor> mDescriptors;
    FeatureExtractor* extractor;
    if(tracking_state==eTrackingState::INITIALIZATION || tracking_state==eTrackingState::NO_IMAGES_YET){
        extractor = extractor_init;
    }
    else {
        extractor = extractor_left;
    }
    (*extractor)(mImGray      , cv::Mat(), mvKeys     ,  mDescriptors );
    FeatureExtractorSettings orb_params;// = setFeatureExtractorSettings(extractor);
    FeatureViews LMviews(mvKeys, mDescriptors,  orb_params);

    ImageFeatureData track_data;
    track_data.img_data = img_data;
    track_data.sensor_data = sensor_data;
    track_data.image = mImGray;
    track_data.LMviews = LMviews;

    output_queue->push(track_data);

}

void ImageProcessing::ProcessStereoImage(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data,  eTrackingState tracking_state){
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    n_processed++;
    cam_cur = img_data.camera;
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    mImGray     = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);
    imGrayRight = PreProcessImg(imGrayRight, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<FeatureDescriptor> mDescriptors;
    std::vector<FeatureDescriptor> mDescriptorsRight;
    std::thread orb_thread(FeatureUtil::extractFeatures, extractor_left, std::ref(mImGray), std::ref(mvKeys), std::ref(mDescriptors) );
    (*extractor_right)(imGrayRight, cv::Mat(), mvKeysRight, mDescriptorsRight );
    orb_thread.join();
    FeatureExtractorSettings orb_params;// = setFeatureExtractorSettings(mpORBextractorLeft);

    if((n_processed % 20) == 0 ) {
        cv::Mat imCopy = imGrayRight.clone();
        cv::drawKeypoints(imCopy, mvKeysRight, imCopy, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite("features_extracted_right.jpg", imCopy);
     //   std::cout << "N Features Extracted: " << mvKeys.size() << std::endl;


        cv::Mat imCopyLeft = mImGray.clone();
        cv::drawKeypoints(imCopyLeft, mvKeys, imCopyLeft, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite("features_extracted_left.jpg", imCopyLeft);
    }

    FeatureViews LMviews(mvKeys, mvKeysRight, mDescriptors, mDescriptorsRight, orb_params);
    Stereomatcher stereomatch( LMviews, cam_data[cam_cur], feature_settings);
    stereomatch.computeStereoMatches();
    stereomatch.getData(LMviews);

    ImageFeatureData track_data;
    track_data.img_data = img_data;
    track_data.sensor_data = sensor_data;
    track_data.image = mImGray;
    track_data.LMviews = LMviews;

    output_queue->push(track_data);
    std::chrono::steady_clock::time_point t_stop = std::chrono::steady_clock::now();
    std::chrono::duration<int, std::milli> t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_stop-t_start);
   // std::cout << "stereo feature extraction duration (ms):  " << t_elapsed.count() << std::endl;

}

cv::Mat ImageProcessing::PreProcessImg(cv::Mat &img, bool mbRGB, float fscale){

    cv::resize(img, img, cv::Size(), fscale, fscale);

    if(img.channels()==3)
    {
        if(mbRGB)
            cvtColor(img, img ,CV_RGB2GRAY);
        else
            cvtColor(img, img ,CV_BGR2GRAY);
    }
    else if(img.channels()==4)
    {
        if(mbRGB)
            cvtColor(img, img ,CV_RGBA2GRAY);
        else
            cvtColor(img, img ,CV_BGRA2GRAY);
    }

    return img;
}

} //end namespace