/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

/*
 * pangolin (openGL) based drawer to visualize maps. all maps are displayed in a single window.
 * optionally displays: KeyFrames, LandMarks (points), Covisibility Graph (ConnGraph), Spanning tree (SpanTree)
 * and camera trajectories (Trajectory)
 */

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <Trajectory.h>
#include <pangolin/pangolin.h>

#include <mutex>
#include <string>
#include <memory>

namespace HYSLAM
{

struct KeyFrameDrawData{
    bool KFs = true;
    bool connectivity_graph = true;
    bool spanning_tree = true;
};

class MapDrawer
{
public:
    MapDrawer(std::map<std::string, std::shared_ptr<Map>> &_maps, const std::string &strSettingPath);

    std::map<std::string, std::shared_ptr<Map>> maps;

    void DrawMapPoints();
    void DrawKeyFrames( KeyFrameDrawData draw_data );
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawTrajectory(Trajectory* trajectory);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
