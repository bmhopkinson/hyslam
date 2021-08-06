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

#include <vector>
#include <set>
#include <memory>
#include <map>

namespace HYSLAM{

class ImagingFramePlacer {
public:
    ImagingFramePlacer(Camera cam, std::shared_ptr<Trajectory> slam_trajectory_, std::map<std::string, std::shared_ptr<Map> > maps_);
    bool placeImagingFrame(cv::Mat &img, const Imgdata &img_info );
private:
    Camera cam_img;
    std::shared_ptr<Trajectory> slam_trajectory;
    std::map<std::string, std::shared_ptr<Map> > maps;

    KeyFrame* pKF_previous = nullptr;
    std::vector<MapPoint*> mpts_previous;
    std::set<KeyFrame*> retained_keyframes;
    double overlap_threshold = 0.8; //if fraction of mapts viewed between current frame and previous retained frame passes below this threshold, keep current frame

    double overlapWithPreviousFrame(KeyFrame* pKF, KeyFrame* pKF_previous, std::vector<MapPoint*> mpts_previous);
};

}
#endif //HYSLAM_IMAGINGFRAMEPLACER_H
