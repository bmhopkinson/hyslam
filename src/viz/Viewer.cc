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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace HYSLAM
{

Viewer::Viewer(System* pSystem, std::map<std::string, FrameDrawer*> pFrameDrawers, MapDrawer *pMapDrawer, Tracking *pTracking, const std::string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawers(pFrameDrawers),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowConnGraph("menu.Show ConnGraph",true,true);
    pangolin::Var<bool> menuShowSpanTree("menu.Show SpanTree",true,true);
    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    for(std::map<std::string, FrameDrawer*>::iterator it = mpFrameDrawers.begin(); it != mpFrameDrawers.end(); it++){
        cv::namedWindow(it->first);
    }

    bool bFollow = true;

    while(1)
    {
        std::cout << "top of viewer while loop " << std::endl;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

     //   mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
        std::cout << "got openGLcamera matrix " << std::endl;
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }
        std::cout << "about to d_cam.Activate " << std::endl;
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
     //   mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowConnGraph || menuShowSpanTree ) {
            KeyFrameDrawData draw_data;
            draw_data.KFs = menuShowKeyFrames;
            draw_data.connectivity_graph = menuShowConnGraph;
            draw_data.spanning_tree = menuShowSpanTree;
         //   mpMapDrawer->DrawKeyFrames(draw_data);
        }
        std::cout << "about to draw mappoints nad trajectory " << std::endl;
        if(menuShowPoints)
        //    mpMapDrawer->DrawMapPoints();
        if(menuShowTrajectory)
       //     mpMapDrawer->DrawTrajectory(mpTracker->trajectories.at("SLAM").get());

        pangolin::FinishFrame();
        
        for(std::map<std::string, FrameDrawer*>::iterator it = mpFrameDrawers.begin(); it != mpFrameDrawers.end(); it++){
            std::cout << "drawing frame for " << it->first <<std::endl;
            FrameDrawer* drawer = it->second;
            cv::Mat im = drawer->DrawFrame();
            cv::imshow(it->first, im);
            cv::waitKey(mT);
        }
        
        if(menuReset)
        {
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuShowConnGraph = true;
            menuShowSpanTree = true;
            menuShowTrajectory = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->RequestReset();
            menuReset = false;
        }

        if(Stop())
        {
            std::cout << "Viewer stopping " << std::endl;
            while(isStopped())
            {
                usleep(3000);
            }
            std::cout << "Viewer restarting " << std::endl;
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    std::unique_lock<std::mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopped = false;
}

}
