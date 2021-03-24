//
// Created by cv-bhlab on 3/3/21.
//

#include "ImageProcessing.h"
#include <ORBFinder.h>
#include <SURFFinder.h>
#include <FeatureViews.h>
#include <ORBUtil.h>
#include <Stereomatcher.h>



#include <iostream>
#include <thread>

namespace HYSLAM{
ImageProcessing::ImageProcessing(FeatureFactory* factory, const std::string &strSettingPath, std::map<std::string, Camera> cam_data_)
 : cam_data(cam_data_)
{
    // Load camera parameters from settings file
    FeatureExtractorSettings feature_extractor_settings;

    std::string tracking_config_file;
    LoadSettings(strSettingPath, feature_extractor_settings, feature_settings);

    dist_func = std::make_shared<ORBDistance>();
    
    //mpORBextractorLeft = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), dist_func, ORBextractor_settings);
    //mpORBextractorRight = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), dist_func, ORBextractor_settings);
  //  SURFextractor = new FeatureExtractor(std::make_unique<SURFFinder>(), ORBextractor_settings);

    mpORBextractorLeft = factory->getExtractor(feature_extractor_settings);
    mpORBextractorRight = factory->getExtractor(feature_extractor_settings);

    FeatureExtractorSettings feature_extractor_settings_init;
    feature_extractor_settings_init = feature_extractor_settings;
    feature_extractor_settings_init.nFeatures = 3 * feature_extractor_settings.nFeatures;
    mpIniORBextractor = factory->getExtractor( feature_extractor_settings_init );
   // mpIniORBextractor = new FeatureExtractor(std::make_unique<ORBFinder>(20.0, true), dist_func, ORBextractor_settings_init);

   dist_func_surf = std::make_shared<SURFDistance>();
   SURFextractor = new SURFExtractor(std::make_unique<SURFFinder>(), dist_func_surf, feature_extractor_settings);
}


void ImageProcessing::ProcessMonoImage(const cv::Mat &im, const Imgdata img_data, const SensorData &sensor_data, eTrackingState tracking_state){
    cam_cur = img_data.camera;
    mImGray = im;
    mImGray = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<FeatureDescriptor> mDescriptors;
    FeatureExtractor* extractor;
    if(tracking_state==eTrackingState::INITIALIZATION || tracking_state==eTrackingState::NO_IMAGES_YET){
        extractor = mpIniORBextractor;
    }
    else {
        extractor = mpORBextractorLeft;
    }
    (*extractor)(mImGray      , cv::Mat(), mvKeys     ,  mDescriptors );
    ORBExtractorParams orb_params = setORBExtractorParams(extractor);
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
    cam_cur = img_data.camera;
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    mImGray     = PreProcessImg(mImGray, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);
    imGrayRight = PreProcessImg(imGrayRight, cam_data[cam_cur].RGB, cam_data[cam_cur].scale);

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<FeatureDescriptor> mDescriptors;
    std::vector<FeatureDescriptor> mDescriptorsRight;
    std::thread orb_thread(ORBUtil::extractORB, mpORBextractorLeft, std::ref(mImGray), std::ref(mvKeys),std::ref( mDescriptors) );
    (*mpORBextractorRight)(imGrayRight, cv::Mat(), mvKeysRight,  mDescriptorsRight );
    orb_thread.join();
    ORBExtractorParams orb_params = setORBExtractorParams(mpORBextractorLeft);

  //  for(auto it = mvKeys.begin(); it != mvKeys.end(); ++it){
 //       cv::KeyPoint kpt = *it;
 //       std::cout << kpt.pt << " , octave: "<< kpt.octave <<  ", size: " << kpt.size << " , scale_factor: " << orb_params.mvScaleFactors[kpt.octave] <<std::endl;
 //   }

    std::vector<cv::KeyPoint> surf_keys;
    std::vector<FeatureDescriptor> surf_descriptors;
  //  (*SURFextractor)(imGrayRight, cv::Mat(), surf_keys,  surf_descriptors );
   // cv::Mat imCopy = imGrayRight.clone();
   // cv::drawKeypoints(imCopy, surf_keys, imCopy, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //cv::imwrite("surf_test.jpg", imCopy);
   // std::cout << "N SURF Features: " << surf_keys.size() << std::endl;
 //   for(auto it = surf_keys.begin(); it != surf_keys.end(); ++it){
 //       cv::KeyPoint kpt = *it;
 //      std::cout << kpt.pt << "octave: "<< kpt.octave <<  ", size: " << kpt.size << std::endl;
 //   }


    FeatureViews LMviews(mvKeys, mvKeysRight, mDescriptors, mDescriptorsRight, orb_params);
    DescriptorDistance* dist_calc = new ORBDistance();
    Stereomatcher stereomatch(mpORBextractorLeft, mpORBextractorRight, LMviews, dist_calc, cam_data[cam_cur], feature_settings);
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

void ImageProcessing::LoadSettings(std::string settings_path, FeatureExtractorSettings &feature_extractor_settings, FeatureMatcherSettings &feature_settings){

    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    feature_extractor_settings.nFeatures = fSettings["ORBextractor.nFeatures"];
    feature_extractor_settings.fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    feature_extractor_settings.nLevels = fSettings["ORBextractor.nLevels"];
    feature_extractor_settings.init_threshold = fSettings["ORBextractor.iniThFAST"];
    feature_extractor_settings.min_threshold = fSettings["ORBextractor.minThFAST"];

    feature_settings.TH_HIGH = fSettings["ORBextractor.match_thresh_high"];
    feature_settings.TH_LOW  = fSettings["ORBextractor.match_thresh_low"];

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << feature_extractor_settings.nFeatures << std::endl;
    std::cout << "- Scale Levels: " << feature_extractor_settings.nLevels << std::endl;
    std::cout << "- Scale Factor: " << feature_extractor_settings.fScaleFactor << std::endl;
    std::cout << "- Initial Extractor Threshold: " << feature_extractor_settings.init_threshold << std::endl;
    std::cout << "- Minimum Extractor Threshold: " << feature_extractor_settings.min_threshold << std::endl;

    std::cout << "matching threshold HIGH: " << feature_settings.TH_HIGH << std::endl;
    std::cout << "matching threshold LOW: " << feature_settings.TH_LOW<< std::endl;
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