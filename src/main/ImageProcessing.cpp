//
// Created by cv-bhlab on 3/3/21.
//

#include "ImageProcessing.h"
#include <ORBFinder.h>
#include <ORBViews.h>
#include <ORBUtil.h>
#include <ORBstereomatcher.h>


#include <iostream>
#include <thread>

namespace HYSLAM{
ImageProcessing::ImageProcessing(const std::string &strSettingPath, Tracking* pTracker, std::map<std::string, Camera> cam_data_)
 : mpTracker(pTracker), cam_data(cam_data_)
{
    // Load camera parameters from settings file
    ORBextractorSettings ORBextractor_settings;
    std::string tracking_config_file;
    LoadSettings(strSettingPath, ORBextractor_settings);
    
    mpORBextractorLeft = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), ORBextractor_settings);
    mpORBextractorRight = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), ORBextractor_settings);

    ORBextractorSettings ORBextractor_settings_init;
    ORBextractor_settings_init = ORBextractor_settings;
    ORBextractor_settings_init.nFeatures = 3 * ORBextractor_settings.nFeatures;
    mpIniORBextractor = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), ORBextractor_settings_init);
}


void ImageProcessing::ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state){
    cam_cur = img_data.camera;
    mImGray = im;
    mImGray = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    FeatureExtractor* extractor;
    if(tracking_state==eTrackingState::INITIALIZATION || tracking_state==eTrackingState::NO_IMAGES_YET){
        extractor = mpIniORBextractor;
    }
    else {
        extractor = mpORBextractorLeft;
    }
    (*extractor)(mImGray      , cv::Mat(), mvKeys     ,  mDescriptors );
    ORBExtractorParams orb_params = setORBExtractorParams(extractor);
    ORBViews LMviews(mvKeys, mDescriptors,  orb_params);

    ImageFeatureData track_data;
    track_data.img_data = img_data;
    track_data.sensor_data = sensor_data;
    track_data.image = mImGray;
    track_data.LMviews = LMviews;

    //mpTracker->track(track_data);
    output_queue->push(track_data);

}

void ImageProcessing::ProcessStereoImage(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const Imgdata img_data,  const  SensorData &sensor_data,  eTrackingState tracking_state){
    cam_cur = img_data.camera;
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    mImGray     = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);
    imGrayRight = PreProcessImg(imGrayRight, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    cv::Mat mDescriptors, mDescriptorsRight;
    std::thread orb_thread(ORBUtil::extractORB, mpORBextractorLeft, std::ref(mImGray), std::ref(mvKeys),std::ref( mDescriptors) );
    (*mpORBextractorRight)(imGrayRight, cv::Mat(), mvKeysRight,  mDescriptorsRight );
    orb_thread.join();
    ORBExtractorParams orb_params = setORBExtractorParams(mpORBextractorLeft);

    ORBViews LMviews(mvKeys, mvKeysRight, mDescriptors, mDescriptorsRight, orb_params);
    ORBstereomatcher stereomatch(mpORBextractorLeft, mpORBextractorRight, LMviews, cam_data[cam_cur]);
    stereomatch.computeStereoMatches();
    stereomatch.getData(LMviews);

    ImageFeatureData track_data;
    track_data.img_data = img_data;
    track_data.sensor_data = sensor_data;
    track_data.image = mImGray;
    track_data.LMviews = LMviews;

    //mpTracker->track(track_data);
    output_queue->push(track_data);

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

void ImageProcessing::LoadSettings(std::string settings_path, ORBextractorSettings &ORBext_settings){
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    ORBext_settings.nFeatures = fSettings["ORBextractor.nFeatures"];
    ORBext_settings.fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    ORBext_settings.nLevels = fSettings["ORBextractor.nLevels"];
    ORBext_settings.fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    ORBext_settings.fMinThFAST = fSettings["ORBextractor.minThFAST"];
    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << ORBext_settings.nFeatures << std::endl;
    std::cout << "- Scale Levels: " << ORBext_settings.nLevels << std::endl;
    std::cout << "- Scale Factor: " << ORBext_settings.fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << ORBext_settings.fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << ORBext_settings.fMinThFAST << std::endl;

    fSettings.release();

}

ORBExtractorParams ImageProcessing::setORBExtractorParams(FeatureExtractor* extractor){
    ORBExtractorParams params;

    params.mnScaleLevels = extractor->GetLevels();
    params.mfScaleFactor = extractor->GetScaleFactor();
    params.mfLogScaleFactor = log(params.mfScaleFactor);
    params.mvScaleFactors = extractor->GetScaleFactors();
    params.mvInvScaleFactors = extractor->GetInverseScaleFactors();
    params.mvLevelSigma2 = extractor->GetScaleSigmaSquares();
    params.mvInvLevelSigma2 = extractor->GetInverseScaleSigmaSquares();

    return params;
}

} //end namespace