//
// Created by hopkinsonlab on 8/5/21.
//

#include "ImagingFramePlacer.h"


namespace HYSLAM{

ImagingFramePlacer::ImagingFramePlacer(Camera cam, std::shared_ptr<Trajectory> slam_trajectory_,
                                       std::map<std::string, std::shared_ptr<Map>> maps_):
                                       cam_img(cam), slam_trajectory(slam_trajectory_), maps(maps_)
{
    if(cam_img.sensor == 1){
        is_stereo = true;
    }

}

bool ImagingFramePlacer::placeImagingFrame(cv::Mat &img, const Imgdata &img_info) {

    //extract relevant info
    double time_stamp = img_info.time_stamp;

    //create keyframe
    //determine pose of imaging camera at this frame based on time and SLAM trajectory
    cv::Mat Tslam;
    if(!slam_trajectory->poseAtTime(time_stamp, Tslam)){
        std::cout <<"not placing image b/c could not determine SLAM cam pose" << std::endl;
        return false;
    }
    cv::Mat Timg = cam_img.Tcam.inv() * Tslam;

    Frame frame(time_stamp, cam_img, img_info.name, is_stereo);
    frame.SetPose(Timg); //need to have a pose or KeyFrame constructor will segfault
    KeyFrame* pKF = new KeyFrame(frame); //shared_ptr would be preferrable

    if(pKF_previous){
        double overlap = overlapWithPreviousFrame(pKF,pKF_previous,mpts_previous);
        if(overlap < overlap_threshold){
            std::vector<MapPoint*> visible_mpts;
            maps["SLAM"]->visibleMapPoints(pKF, visible_mpts);
            if(visible_mpts.size() > min_mpts){
                retainKeyFrame(pKF, visible_mpts);
               // std::cout << "placing imaging frame due to low overlap" << std::endl;
                return true;
            } else {
                pKF_previous = nullptr;
              //  std::cout << "not placing imaging frame due to low # visible mappoints" << std::endl;
            }
        } else {
            //std::cout << "not placing frame due to sufficient overlap: " << overlap << " , based on how many previous mappoints: " << mpts_previous.size() << std::endl;
        }
    } else {
        std::vector<MapPoint*> visible_mpts;
        maps["SLAM"]->visibleMapPoints(pKF, visible_mpts);
        if(visible_mpts.size() > min_mpts){
            retainKeyFrame(pKF, visible_mpts);
            //std::cout << "placing imaging frame, no previous frame and sufficient mappoints visible " << std::endl;
            return true;
        } else {
            //std::cout << "not placing imaging frame due to low # visible mappoints" << std::endl;
        }
    }

    return false;
}

double ImagingFramePlacer::overlapWithPreviousFrame(KeyFrame *pKF, KeyFrame *pKF_previous, std::vector<MapPoint*> mpts_previous) {
    int n_vis = 0;
    for(auto it = mpts_previous.begin(); it != mpts_previous.end(); ++it){
        MapPoint* pMP = *it;
        if(pKF->isLandMarkVisible(pMP)){
            n_vis++;
        }
    }
    double overlap = static_cast<double>(n_vis)/static_cast<double>(mpts_previous.size());
    return overlap;
}

void ImagingFramePlacer::retainKeyFrame(KeyFrame *pKF, std::vector<MapPoint *> mpts_visible) {
    pKF_previous = pKF;
    mpts_previous = mpts_visible;
    retained_keyframes.insert(pKF);
    maps["Imaging"]->AddKeyFrame(pKF); //allows retained keyframes to be exported (colamp, agisoft) and KFs will be visualized
}

void ImagingFramePlacer::setOverlapThreshold(double overlapThreshold) {
    overlap_threshold = overlapThreshold;
}

void ImagingFramePlacer::setMinMpts(int minMpts) {
    min_mpts = minMpts;
}

}//close namespace