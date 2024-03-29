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

#include "FrameDrawer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace HYSLAM
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=eTrackingState::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    std::vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    std::vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    std::vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    eTrackingState state; // Tracking state
    FeatureViews views_current;
    LandMarkMatches lm_matches_current;

    //Copy variables within scoped mutex
    {
        std::unique_lock<std::mutex> lock(mMutex);
        state=mState;
        if(mState==eTrackingState::SYSTEM_NOT_READY)
            mState=eTrackingState::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==eTrackingState::INITIALIZATION)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==eTrackingState::NORMAL)
        {

            views_current = views;
            lm_matches_current = lm_matches;
        }
        else if(mState==eTrackingState::RELOCALIZATION)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==eTrackingState::INITIALIZATION) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==eTrackingState::NORMAL) //TRACKING
    {
        mnTracked=0;
        for(auto it = lm_matches_current.begin(); it != lm_matches_current.end(); ++it)
        {
            size_t idx = it->first;
            MapPoint* lm = it->second;
            int n_frames_tracked = lm_matches_current.getNumFramesTracked(lm);

            cv::KeyPoint keypt = views_current.keypt(idx);
            cv::Point2f pt1,pt2;
            float feature_scale = 0.5; //0.5 provides some indication of size of feature but isn't as distracting as 1.0
            float r = feature_scale * keypt.size/2.0;
            pt1.x=keypt.pt.x-r;
            pt1.y=keypt.pt.y-r;
            pt2.x=keypt.pt.x+r;
            pt2.y=keypt.pt.y+r;

            cv::Scalar color_rect;
            if(n_frames_tracked>20) {
                color_rect = cv::Scalar(255, 0, 0);
            } else if (n_frames_tracked>5){
                color_rect = cv::Scalar(255, 255, 0);
            }
            else {
                color_rect = cv::Scalar(0, 255, 0);
            }

            cv::Scalar color_circle;
            int n_lm_obs = lm->Observations();
            if(n_lm_obs > 100) {
                color_circle = cv::Scalar(255, 0, 0);
            } else if (n_lm_obs > 30){
                color_circle = cv::Scalar(255, 255, 0);
            }
            else {
                color_circle = cv::Scalar(0, 255, 0);
            }

            cv::rectangle(im, pt1, pt2, color_rect);
            cv::circle(im,keypt.pt,3,color_circle,-1);
            mnTracked++;
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, eTrackingState nState, cv::Mat &imText)
{
    std::stringstream s;
    if(nState==eTrackingState::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==eTrackingState::INITIALIZATION)
        s << " TRYING TO INITIALIZE ";
    else if(nState==eTrackingState::NORMAL)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
    }
    else if(nState==eTrackingState::RELOCALIZATION)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==eTrackingState::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    std::unique_lock<std::mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mbOnlyTracking = false;

    if(pTracker->mLastProcessedState==eTrackingState::INITIALIZATION)
    {
        mvCurrentKeys = pTracker->mCurrentFrame.getViews().getKeys();
        mvIniKeys=pTracker->init_data[pTracker->cam_cur].mInitialFrame.getViews().getKeys();
        mvIniMatches=pTracker->init_data[pTracker->cam_cur].mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==eTrackingState::NORMAL)
    {
        lm_matches =  pTracker->mCurrentFrame.getLandMarkMatches();
        views = pTracker->mCurrentFrame.copyViews();
    }
    mState=pTracker->mLastProcessedState;
}

void FrameDrawer::clear(){
    mvCurrentKeys = std::vector<cv::KeyPoint>();
    mvIniKeys= std::vector<cv::KeyPoint>();
    mvIniMatches = std::vector<int>();
     views= FeatureViews();
     lm_matches = LandMarkMatches();
}

} //namespace ORB_SLAM
