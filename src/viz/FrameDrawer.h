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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

/*
 * class to display images and information about feature tracking (keypoints, initialization matches, etc)
 * unfortunate dependancy on Tracking should be eliminated.
 *
 * Displays each frame the SLAM system attempts to track. for tracked frames it displays markers indicating the tracked
 * features (i.e. features associated with landmarks) with the marker size proportional to the feature size.
 *  the color of the outer square indicates how many consecutive frames the landmark has been tracked in (currently: <5 = green; 5 - 20 = cyan; >20 blue).
 *  the color of the point at the center of the marker indicates the total number of keyframes in which the landmark has been observed
 *   (currently: <30 = green, 30 -100 = cyan; >100 = blue)
 *  Along the bottom of the image indicates information about the map associated with the camera: total keyframes, total mappoints (landmarks)
 *  and current number of feature to landmark matches.
 *
 *  Key functionality:
 *   cv::Mat DrawFrame() - creates and returns annotated image of most recent frame for display
 *
 *   void Update(Tracking *pTracker) - pulls image and tracking data of most recent frame for processing
 */

#include <Tracking.h>
#include <Tracking_datastructs.h>
#include <MapPoint.h>
#include <Map.h>
#include <FeatureViews.h>
#include <LandMarkMatches.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <vector>


namespace HYSLAM
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();

    void clear();

protected:

    void DrawTextInfo(cv::Mat &im, eTrackingState nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;

    bool mbOnlyTracking;
    int mnTracked;
    std::vector<cv::KeyPoint> mvCurrentKeys;
    std::vector<cv::KeyPoint> mvIniKeys;
    std::vector<int> mvIniMatches;
    FeatureViews views;
    LandMarkMatches lm_matches;
    eTrackingState mState;

    Map* mpMap;
    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
