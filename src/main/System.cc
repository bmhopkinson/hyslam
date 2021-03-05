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



#include "System.h"
#include "Converter.h"
#include "Trajectory.h"
#include <MapPointDB.h>
#include <Camera.h>
#include "ORBSLAM_datastructs.h"
#include <ImagingBundleAdjustment.h>

#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <string>
#include <time.h>

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}


namespace HYSLAM
{

System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false)
{
    std::cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        std::cout << "Monocular" << std::endl;
    else if(mSensor==STEREO)
        std::cout << "Stereo" << std::endl;
    else if(mSensor==RGBD)
        std::cout << "RGB-D" << std::endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
       exit(-1);
    }

    LoadSettings(strSettingsFile);
    mpVocabulary = new ORBVocabulary();
    LoadVocabulary( strVocFile, mpVocabulary);

    //Load Camera data and create per camera data structures
    cv::FileNode cameras = fsSettings["Cameras"];
    for(cv::FileNodeIterator it = cameras.begin(); it != cameras.end(); it++){
        cv::FileNode camera = *it;
        std::string cam_name  = (*it).name();

        //load camera data
        Camera cam_info;
        cam_info.loadData(camera);
        cam_data.insert( std::make_pair(cam_name, cam_info) );
        current_tracking_state[cam_name] = eTrackingState::SYSTEM_NOT_READY;
        std::cout << "system loadsettings: cameras: " << cam_name << std::endl;

        //create map
        Map* mpMap = new Map();
        mpMap->setKeyFrameDBVocab(mpVocabulary);
        maps.insert(std::make_pair(cam_name, mpMap));

        //create frame drawer
        FrameDrawer* pFrameDrawer = new FrameDrawer(maps[cam_name]);
        mpFrameDrawers.insert(std::make_pair( cam_name ,pFrameDrawer) );

    }

    //Create Drawers. These are used by the Viewer
    mpMapDrawer = new MapDrawer(maps, strSettingsFile);
    
    //Initialize Main Threads
    //Shared Data structures
    thread_status = std::make_unique<MainThreadsStatus>();

    //Initialize Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawers, mpMapDrawer,
                             maps, cam_data, strSettingsFile, thread_status.get());

    mpImageProcessor = new ImageProcessing(strSettingsFile, mpTracker, cam_data);

    //Initialize the Local Mapping thread and launch
    std::string mapping_config_path = fsSettings["Mapping_Config"].string();
    mpLocalMapper = new Mapping(maps, mSensor==MONOCULAR, mapping_config_path, thread_status.get());
    mptLocalMapping = new std::thread(&HYSLAM::Mapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(maps, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new std::thread(&HYSLAM::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawers,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new std::thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const Imgdata &img_info,  const SensorData &sensor_data)
{
    std::string cam_cur = img_info.camera;
    cv::Mat Tcw;

   // cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight, img_info, sensor_data);
    mpImageProcessor->ProcessStereoImage(imLeft,imRight,img_info, sensor_data, current_tracking_state[cam_cur]);
    current_tracking_state[cam_cur] = mpTracker->GetCurrentTrackingState(cam_cur);
    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackedMapPoints.clear();
    mTrackedKeyPointsUn.clear();
//    mpTracker->mCurrentFrame.getAssociatedLandMarks(mTrackedKeyPointsUn, mTrackedMapPoints);
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const Imgdata &img_info, const SensorData &sensor_data)
{
    std::string cam_cur = img_info.camera;
    cv::Mat Tcw;

    //cv::Mat Tcw = mpTracker->GrabImageMonocular(im,img_info, sensor_data);
    mpImageProcessor->ProcessMonoImage(im, img_info, sensor_data, current_tracking_state[cam_cur]);
    current_tracking_state[cam_cur] = mpTracker->GetCurrentTrackingState(cam_cur);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackedMapPoints.clear();
    mTrackedKeyPointsUn.clear();
  //  mpTracker->mCurrentFrame.getAssociatedLandMarks(mTrackedKeyPointsUn, mTrackedMapPoints);

    return Tcw;
}

void System::RunImagingBundleAdjustment(){
   // mpTracker->RunImagingBundleAdjustment();
    //stop LocalMapping and LoopClosing
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    while(mpLoopCloser->isRunningGBA()){  //can't stop loop closing right now - add this capability  -just check to make sure a GBA isn't running
        usleep(10000);
    }

    g2o::Trajectory traj_g2o = mpTracker->trajectories["SLAM"]->convertToG2O();
    ImagingBundleAdjustment imgBA(maps["Imaging"], mpTracker->trajectories["Imaging"].get(), traj_g2o, optParams );
    imgBA.Run();

    mpLocalMapper->Release();
}

void System::Reset()
{
    /// THIS HASN'T BEEN TESTED SO PROBABLY DOESN:T WORK
    std::unique_lock<std::mutex> lock(mMutexReset);

    std::cout << "System Reseting" << std::endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    mpTracker->Reset();

    // Reset Local Mapping
    std::cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    std::cout << " done" << std::endl;

    // Reset Loop Closing
    std::cout << "Reseting Loop Closing...";
    mpLoopCloser->RequestReset();
    std::cout << " done" << std::endl;


    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;

    if(mpViewer)
        mpViewer->Release();

}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

std::vector<int> System::ValidImagingKeyFrames(){
  std::vector<int> KF_frame_numbers;
  std::vector<KeyFrame*> vpKFs = maps["Imaging"]->GetAllKeyFrames();
  for(std::vector<KeyFrame*>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit){
    KeyFrame* pKF = *vit;
    if(pKF->isBad()){
      continue;
    }
    else{
      KF_frame_numbers.push_back( std::stoi(pKF->kfImgName) );
    }
  }

  return KF_frame_numbers;

}




void System::SaveTrajectoryMapping(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;

    std::vector<KeyFrame*> vpKFs = maps["SLAM"]->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    std::vector<cv::Mat> poses = mpTracker->trajectories["SLAM"]->getPoses(true);
    std::vector<cv::Mat>::iterator pit = poses.begin();

    std::vector<TrajectoryElement> trajectory_elements = mpTracker->trajectories["SLAM"]->getTrajectoryElements();
    for(std::vector<TrajectoryElement>::iterator it = trajectory_elements.begin(); it != trajectory_elements.end(); it++, pit++){
        cv::Mat pose = (*pit)*Two;
        cv::Mat Rwc =  pose.rowRange(0,3).colRange(0,3);  //camera to world rotation
        cv::Mat twc =  pose.rowRange(0,3).col(3);    // camera to world translation
        std::string name = (*it).name;
        double time = (*it).time_stamp;
  /////WORKIGN HERE - verified this all looks correct. need to add frame number and timestamp for each frame
        f << name << "\t"  << std::setprecision(9) <<  time <<"\t" << Rwc.at<float>(0,0) << "\t" << Rwc.at<float>(0,1)  << "\t" << Rwc.at<float>(0,2) << "\t"  << twc.at<float>(0) << "\t" <<
             Rwc.at<float>(1,0) << "\t" << Rwc.at<float>(1,1)  << "\t" << Rwc.at<float>(1,2) << "\t"  << twc.at<float>(1) << "\t" <<
             Rwc.at<float>(2,0) << "\t" << Rwc.at<float>(2,1)  << "\t" << Rwc.at<float>(2,2) << "\t"  << twc.at<float>(2) << std::endl;
    }
    f.close();
    std::cout << std::endl << "trajectory saved!" << std::endl;

}


void System::ExportCOLMAP(const std::string &foldername){
  struct stat sb;
  std::ios_base::fmtflags fmt_flags( std::cout.flags() ); // save fmt flags to restore after manipulation

  if(stat(foldername.c_str(), &sb) == -1){
    perror("stat");
    exit(EXIT_FAILURE);
  }

  if(!S_ISDIR(sb.st_mode)){
    if( mkdir(foldername.c_str(), S_IRUSR | S_IWUSR ) != 0) {
      std::cout << "could not create: " << foldername << std::endl;
      return;
    }
  }
  //ULTIMATELY WILL NEED TO ACCOUNT FOR IMAGE SCALING !!!!!!

  //COLMAP required file names
  std::string cam_file_name  = "cameras.txt";
  std::string imgs_file_name = "images.txt";
  std::string pts_file_name  = "points3D.txt";

  //camera file
    std::ofstream f;
  f.open(cam_file_name.c_str()); //add folder path at some point
    std::vector<KeyFrame*> vpKFs = maps["Imaging"]->GetAllKeyFrames();
  Camera camera_data  = vpKFs[0]->camera;
  f << "# Camera list with one line of data per camera:" << "\n"  //header
    << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]"   << "\n"
    << "# Number of cameras: 1" << std::endl;

  f << "1 OPENCV "; //currently only have one camera and the camer std::setprecision(1) <<a model is the OPENCV model
  int dim_x = vpKFs[0]->mnMaxX;
  int dim_y = vpKFs[0]->mnMaxY;
  f << dim_x << " " << dim_y << " ";

  float fx, fy, cx, cy;  //focal length and principal point
  fx = camera_data.fx();
  fy = camera_data.fy();
  cx = camera_data.cx();
  cy = camera_data.cy();
  f << fx << " " << fy << " " << cx << " " << cy << " ";

  float k1, k2, p1, p2;  //radial and tangential distortion
  k1 = camera_data.distCoef.at<float>(0);
  k2 = camera_data.distCoef.at<float>(1);
  p1 = camera_data.distCoef.at<float>(2);
  p2 = camera_data.distCoef.at<float>(3);
  f << std::fixed << std::setprecision(6) << k1 << " " << k2 << " " << p1 << " "  << p2 << std::endl;
  f.close();

  //find optimized Tslam_to_img
  cv::Mat Tcam_opt;
  for(std::vector<KeyFrame*>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit){
    KeyFrame* pKFi = *vit;
    if(pKFi->isBad()){
      continue;
    }

    if( !pKFi->camera.Tcam_opt.empty() ){
      Tcam_opt = pKFi->camera.Tcam_opt;
    }

  }

  //image file - keyframes and KeyPoints
  f.open(imgs_file_name.c_str());
  for(std::vector<KeyFrame*>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit){
    KeyFrame* pKF = *vit;
    if(pKF->isBad()){
      continue;
    }
  //  cv::Mat Twc = pKF->GetPoseInverse(); //pose in camera to world convention
  //  Twc = Tcam_opt * Twc; // chain

    f << pKF->mnId << " ";

    std::vector<float> quat = Converter::toQuaternion(pKF->GetRotation());  //rotation is world to camera
    float qw, qx, qy, qz;
    qw = quat[3];
    qx = quat[0];
    qy = quat[1];
    qz = quat[2];
    f << std::fixed << std::setprecision(5) << qw << " " << qx << " " << qy << " " << qz << " ";

    cv::Mat t_cw =  pKF->GetTranslation();  //world to camera translation
    f << std::fixed << std::setprecision(4) << t_cw.at<float>(0) << " " << t_cw.at<float>(1) << " " << t_cw.at<float>(2) << " ";

    f << "1" << " "; //camera id - right now assuming only one camera
    std::string img_file_name = "imgcam_" + pKF->kfImgName + ".jpg";
    f << img_file_name << std::endl;


    //now write out keypoints and corresponding mappt ids - all pts from a single keyframe go on one line
    std::vector<MapPoint*> mappts_indexed = pKF->GetMapPointMatches();
    const ORBViews views = pKF->getViews();
    std::vector<cv::KeyPoint> keypts_indexed = views.getKeys();
    int n_keypts = keypts_indexed.size();
    for(int i = 0; i < n_keypts; ++i){
      if(!mappts_indexed[i]){
        continue;  //no mappt associated with this keypoint
      }
      if(mappts_indexed[i]->isBad()){
        continue;
      }

      f  <<std::fixed << std::setprecision(1) << keypts_indexed[i].pt.x << " " << keypts_indexed[i].pt.y << " " << mappts_indexed[i]->mnId<< " ";

    }
    f << std::endl;

  } //end loop on keyframes
  f.close();

  //store 3D MapPoints
  f.open(pts_file_name.c_str());
  std::vector<MapPoint*> mpts_all =  maps["Imaging"]->GetAllMapPoints();
  for(std::vector<MapPoint*>::iterator vit = mpts_all.begin(); vit != mpts_all.end(); ++vit){
    MapPoint* pMP = *vit;
    cv::Mat pos = pMP->GetWorldPos();
    f << pMP->mnId << " " << std::fixed << std::setprecision(4) <<  pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << " ";
    f << "125 125 125 0.1 "; //dummy RGB and error values

    //"track" = observations
    std::map<KeyFrame*, size_t> KFobs =  pMP->GetObservations();
    for(std::map<KeyFrame*, size_t>::iterator mit = KFobs.begin(); mit != KFobs.end(); ++mit){
      KeyFrame* pKFobs = (*mit).first;
      size_t idx_obs = (*mit).second;
      f << pKFobs->mnId << " "  << idx_obs << " ";
    }
    f << std::endl;
  } // end loop on mappoints
  f.close();
  std::cout.flags(fmt_flags); //restore format flags

}

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    /*
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list< std::pair<std::string, double> > mlFrameId;

    mlRelativeFramePoses = mpTracker->trajectories["SLAM"].rel_frame_poses;
    mlpReferences = mpTracker->trajectories["SLAM"].refKFs;
    mlFrameId = mpTracker->trajectories["SLAM"].frameids;

    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mlpReferences.begin();
     list< std::pair<std::string, double> >::iterator lT = mlFrameId.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(), lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();  //camera to world rotation
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);    // camera to world translation
  /////WORKIGN HERE - verified this all looks correct. need to add frame number and timestamp for each frame
        f << (*lT).first << "\t"  << setprecision(9) <<  (*lT).second <<"\t" << Rwc.at<float>(0,0) << "\t" << Rwc.at<float>(0,1)  << "\t" << Rwc.at<float>(0,2) << "\t"  << twc.at<float>(0) << "\t" <<
             Rwc.at<float>(1,0) << "\t" << Rwc.at<float>(1,1)  << "\t" << Rwc.at<float>(1,2) << "\t"  << twc.at<float>(1) << "\t" <<
             Rwc.at<float>(2,0) << "\t" << Rwc.at<float>(2,1)  << "\t" << Rwc.at<float>(2,2) << "\t"  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;

}

*/

// MAKE OUTPUT USEFUL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MERGE OPT_UPDATES FIRST
//Flat textfile debugging trajectory -
/*
void System::SaveTrajectoryDebugging(const string &filename){

    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    int i=0;
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
     list< std::pair<std::string, double> >::iterator lT = mpTracker->mlFrameId.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++,i++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        std::vector<float> q = Converter::toQuaternion(Rwc);
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << scientific << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)
                                            << " " << q[3] << " " << q[0] << " "<< q[1] << " "<< q[2] << " ";


    }

    f.close();

    cout << endl << "trajectory saved!" << endl;

}


/*  xml debugging trajectory

void System::SaveTrajectoryDebugging(const string &filename){

    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLElement* pRoot = xmlDoc.NewElement("document");
    pRoot->SetAttribute("version","1.2.0");
    xmlDoc.InsertFirstChild(pRoot);

    tinyxml2::XMLElement* pChunk = xmlDoc.NewElement("chunk");

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    int i=0;
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++,i++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        std::vector<float> q = Converter::toQuaternion(Rwc);
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        tinyxml2::XMLElement* pElement = xmlDoc.NewElement("camera");
		pElement->SetAttribute("id",i);
		string camFile =  mpTracker->mlImgNames[i];
		pElement->SetAttribute("name",camFile.c_str());
		pElement->SetAttribute("sensor_id",0);
		pElement->SetAttribute("enabled","true");

		tinyxml2::XMLElement* pCamElement = xmlDoc.NewElement("transform");
        stringstream ss;
        ss << scientific << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)
                                            << " " << q[3] << " " << q[0] << " "<< q[1] << " "<< q[2] << endl;
        string s = ss.str();
		pCamElement->SetText(s.c_str());
		pElement->InsertEndChild(pCamElement);


		tinyxml2::XMLElement* pCamData = xmlDoc.NewElement("data");
		Eigen::Quaterniond qdata = mpTracker->mvQuats[i];
		stringstream dd;
		dd << setprecision(9) << qdata.w() << " " << qdata.x() << " " << qdata.y() << " " << qdata.z() << endl;
		pCamData->SetText(dd.str().c_str());
		pElement->InsertEndChild(pCamData);

		pChunk->InsertEndChild(pElement);


    }


    pRoot->InsertEndChild(pChunk);
    tinyxml2::XMLError eResult = xmlDoc.SaveFile(filename.c_str());
    cout << endl << "trajectory saved!" << endl;

}
*/
void System::SaveMap(const std::string &filename){
    std::cout << "saving map points" << std::endl;
    std::ofstream f;
    f.open(filename.c_str());

    const std::vector<MapPoint*> &vpMPs = maps["SLAM"]->GetAllMapPoints();
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++){
        if(vpMPs[i]->isBad()) {continue;}
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        f << std::setprecision(6) << pos.at<float>(0) << "\t" << pos.at<float>(1) << "\t" <<pos.at<float>(2) <<std::endl;
    }
    f.close();

}


std::vector<MapPoint*> System::GetTrackedMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

// ADDITION: Tracking state monitoring
bool System::TrackingOk() { return mpTracker->GetCurrentTrackingState() == eTrackingState::NORMAL; }
bool System::TrackingLost() { return mpTracker->GetCurrentTrackingState() == eTrackingState::RELOCALIZATION; }
bool System::TrackingReady() { return (mpTracker->GetCurrentTrackingState() != eTrackingState::SYSTEM_NOT_READY); }
bool System::TrackingNeedImages() { return mpTracker->GetCurrentTrackingState() ==  eTrackingState::NO_IMAGES_YET; }
bool System::TrackingInitialized() { return (mpTracker->GetCurrentTrackingState() == eTrackingState::NORMAL) or (mpTracker->GetCurrentTrackingState() ==  eTrackingState::RELOCALIZATION); }
double System::PercentObserved() { return mpTracker->PercentObserved(); }

void System::LoadSettings(std::string settings_path) {
   // std::cout << "LoadSettings: settings_path " << settings_path << std::endl;
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);
    // load optimizer parameters
    optParams.Info_Depth = fSettings["Opt.Info_Depth"];
    optParams.Info_IMU   = fSettings["Opt.Info_IMU"];
    optParams.Info_GPS   = fSettings["Opt.Info_GPS"];
    int temp = fSettings["Opt.realtime"];
    optParams.realtime = temp; //implicit cast to bool;
    optParams.GBAinterval = fSettings["Opt.GBAinterval"];

}

void System::LoadVocabulary(const std::string vocab_file, ORBVocabulary* vocab){
    clock_t tStart = clock();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(vocab_file, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(vocab_file);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(vocab_file);
    if(!bVocLoad)
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Failed to open at: " << vocab_file << std::endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

} //namespace ORB_SLAM
