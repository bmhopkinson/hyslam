//
// Created by hopkinsonlab on 8/5/21.
//

#ifndef HYSLAM_IMAGINGFRAMEPLACER_H
#define HYSLAM_IMAGINGFRAMEPLACER_H

#include <KeyFrame.h>
#include <MapPoint.h>
#include <Camera.h>
#include <Trajectory.h>
#include <Map.h>
#include <InterThread.h>

#include <opencv2/core/core.hpp>

#include <vector>
#include <set>
#include <memory>
#include <map>

namespace HYSLAM{

class ImagingFramePlacer {
public:
    ImagingFramePlacer(Camera cam, std::shared_ptr<Trajectory> slam_trajectory_, std::map<std::string, std::shared_ptr<Map> > maps_);
    bool placeImagingFrame(cv::Mat &img, const Imgdata &img_info );
    void setOverlapThreshold(double overlapThreshold);
    void setMinMpts(int minMpts);

private:
    Camera cam_img;
    bool is_stereo = false;
    std::shared_ptr<Trajectory> slam_trajectory;
    std::map<std::string, std::shared_ptr<Map> > maps;

    KeyFrame* pKF_previous = nullptr;
    std::vector<MapPoint*> mpts_previous;
    std::set<KeyFrame*> retained_keyframes;
    double overlap_threshold = 0.8;
    //if fraction of mapts viewed between current frame and previous retained frame passes below this threshold, keep current frame
    int min_mpts = 20; // minimum number of landmarks visible in frame needed to retain

    double overlapWithPreviousFrame(KeyFrame* pKF, KeyFrame* pKF_previous, std::vector<MapPoint*> mpts_previous);
    void retainKeyFrame(KeyFrame* pKF, std::vector<MapPoint*> mpts_visible);
};

}
#endif //HYSLAM_IMAGINGFRAMEPLACER_H
