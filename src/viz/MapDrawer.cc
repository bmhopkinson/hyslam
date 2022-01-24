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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <KeyFrameDB.h>
#include <pangolin/pangolin.h>
#include <mutex>

namespace HYSLAM
{


MapDrawer::MapDrawer(std::map<std::string, std::shared_ptr<Map>> &_maps, const std::string &strSettingPath):maps(_maps)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
  for(std::map<std::string, std::shared_ptr<Map>>::const_iterator mit= maps.begin(); mit != maps.end(); ++mit){
    std::string cam_name = (*mit).first;
    std::shared_ptr<Map> mpMap = (*mit).second;
    const std::vector<MapPoint*> &vpMPs = mpMap->getAllMapPointsIncludeSubmaps();
    const std::vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        continue;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(std::set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

  }
  glEnd();
}

void MapDrawer::DrawKeyFrames(KeyFrameDrawData draw_data )
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    for(std::map<std::string, std::shared_ptr<Map>>::const_iterator mit= maps.begin(); mit != maps.end(); ++mit){
      std::string cam_name = (*mit).first;
      std::shared_ptr<Map> mpMap = (*mit).second;

      const std::vector<KeyFrame*> vpKFs = mpMap->getAllKeyFramesIncludeSubmaps();

      if(draw_data.KFs)
      {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
          }
      }

      if(draw_data.connectivity_graph || draw_data.spanning_tree)
      {
        glLineWidth(mGraphLineWidth);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {

            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            // Connectivity Graph
            if(draw_data.connectivity_graph) {
                glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
                const std::vector<KeyFrame *> vCovKFs = mpMap->getBestCovisibilityKeyFrames(vpKFs[i], 4);

                if (!vCovKFs.empty()) {
                    for (std::vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }
            }

            // Spanning tree
            if(draw_data.spanning_tree){
                glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if(pParent) {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }
            }

            // Loops
            std::set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(std::set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }
      }
    }
    glEnd();

}

void MapDrawer::DrawTrajectory(Trajectory* trajectory){
    glLineWidth(2.0*mGraphLineWidth);
    glColor4f(1.0f,0.0f,1.0f,0.6f);
    glBegin(GL_LINES);

    TrajectoryElement te_previous;
    std::lock_guard<std::mutex> lock(trajectory->trajectory_mutex);
    for(auto it = trajectory->cbegin(); it != trajectory->cend(); ++it){
        TrajectoryElement te = *it; //segfault here

        bool plot = true;
        if(it ==trajectory->cbegin() ){
            plot = false;
        }

        if(!te.tracking_good || !te_previous.tracking_good){
            plot = false;
        }

        if(plot) {
            cv::Mat p1 = te.worldPosition();
            cv::Mat p2 = te_previous.worldPosition();
            glVertex3f(p1.at<float>(0), p1.at<float>(1), p1.at<float>(2));
            glVertex3f(p2.at<float>(0), p2.at<float>(1), p2.at<float>(2));
        }

        te_previous = te;
    }

    glEnd();
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
