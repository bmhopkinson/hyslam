
#include <MonoInitializer.h>
#include <FeatureViews.h>
#include <FeatureMatcher.h>

namespace HYSLAM{
MonoInitializer::MonoInitializer(MonoInitializerParameters params_, FeatureFactory* factory): params(params_), feature_factory(factory)
{
    estimator_params.sigma = params.sigma;
    estimator_params.minTriangulated = params.minTriangulated;
    estimator_params.minFracTriangulated = params.minFracTriangulated;
    estimator_params.MaxIterations = params.MaxIterations;
};

int MonoInitializer::initialize(Frame &frame){
    if(!hasValidFirstFrame()){
        firstFrame(frame);//, mono_init_matches[cam_cur]);
        std::cout << "mono init, first frame: " << frame.mnId << std::endl;
        return -1;
    }
        
    else {
        return secondFrame(frame);//, mono_init_matches[cam_cur]);
    }
}

bool MonoInitializer::hasValidFirstFrame(){
    return has_data;
}

int MonoInitializer::firstFrame(Frame &frame){//,  MonoInitialMatch &match_data){
    first_frame = Frame(frame);
    
    const FeatureViews views = first_frame.getViews();
    int n_views = views.numViews();     
        
        // Set Reference Frame
    if(n_views>params.N_min_features)
    {   
     //       mLastFrame[cam_cur] = Frame(mCurrentFrame); CAN'T do this here but still need to do in Tracking I think 
        has_data = true;
            
        mvbPrevMatched.resize(n_views);
        for(size_t i=0; i<n_views; i++){
            mvbPrevMatched[i] = views.keypt(i).pt;
        }

        if(mpInitializer){
            delete mpInitializer;
        }
        mpInitializer =  new MonoEstimator( first_frame ,estimator_params);

        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

        init_data.mInitialFrame = Frame(first_frame);
        init_data.mvIniMatches = mvIniMatches;
        init_data.mvbPrevMatched = mvbPrevMatched;
        init_data.mvIniP3D = mvIniP3D;

        return 0;
    } else {
        has_data = false;
        return -1;
    }
    
}

int MonoInitializer::secondFrame(Frame &frame){//,  MonoInitialMatch &match_data){
    // Try to initialize
    second_frame = frame;
    if(second_frame.getViews().numViews()<=params.N_min_features)
    {
        clear();
        return -1;
    }

    // Find correspondences
   // FeatureMatcher matcher(params.match_nnratio, true);
    FeatureMatcherSettings fm_settings = feature_factory->getFeatureMatcherSettings();
    fm_settings.nnratio = params.match_nnratio;
    fm_settings.checkOri = true;
    feature_factory->setFeatureMatcherSettings(fm_settings);
    std::unique_ptr<FeatureMatcher> matcher = feature_factory->getFeatureMatcher();
    int nmatches = matcher->SearchForInitialization(first_frame, second_frame, mvbPrevMatched, mvIniMatches, 100);
    std::cout << "MonoInit: matches found: " << nmatches << std::endl; // << ", new: "<< nmatches_alt <<  std::endl;
    // Check if there are enough correspondences
    if(nmatches<params.N_min_matches)
    {
        clear();
        return -1;
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    std::vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    
    init_success = mpInitializer->Initialize(second_frame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated);

    init_data.mvIniMatches = mvIniMatches;
    init_data.mvbPrevMatched = mvbPrevMatched;
    init_data.mvIniP3D = mvIniP3D;

    if(init_success)
    {
        std::cout << "successful Mono Init" << std::endl;
        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        {
            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
            {
                mvIniMatches[i]=-1;
                nmatches--;
            }
        }

        // Set Frame Poses
        first_frame.SetPose(cv::Mat::eye(4,4,CV_32F));
        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3));
        second_frame.SetPose(Tcw);

      return 0;
    }
    else{ 
       return 1; 
    }

}


int MonoInitializer::createMap(){    
    // Create KeyFrames
 //   Map* no_map = nullptr;  //not yet associated with a map
    pKFfirst = new KeyFrame(first_frame );
    pKFsecond = new KeyFrame(second_frame);
    
    pKFfirst->ComputeBoW();
    pKFsecond->ComputeBoW();
    
    
     for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos);
        mpts.push_back(pMP);
        
        //associate with KFs and second frame 
        pKFfirst->associateLandMark(i, pMP, true);
        pKFsecond->associateLandMark(mvIniMatches[i], pMP, true);
        pKFfirst->setOutlier(i,false);
        pKFsecond->setOutlier(mvIniMatches[i],false);
        
        second_frame.associateLandMark(mvIniMatches[i], pMP, true);
        second_frame.setOutlier(mvIniMatches[i], false); 
        
        mpts_to_keypts[pMP]  = std::make_pair(i, mvIniMatches[i]);

    }
    map_created = true;
    return 0;
}

int MonoInitializer::createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints){
    if(createMap() == 0){
      pKF1 = pKFfirst;
      pKF2 = pKFsecond;
      mappoints = mpts;
    return 0;
    } else {
        return -1;
    }
    
}

int MonoInitializer::transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T){

    //if not the SLAM camera there's already an existing scale and need to reference to that.
    //one idea - get delta_t between frames used for monocular initialization.
    // get velocity from SLAM (mVelocity but account for delta_t_slam) use this to estimate distance traveled between mono init frames (the baseline),
    // once baseline is established correct map point and keyframe locations - also need to give keyframe and mappoint positions absolute values wrt SLAM cam.

    //place keyframes and mappoints relative to SLAM camera- might be better at end
    cv::Mat Tw1c_init = pKFfirst->GetPose().inv();
    cv::Mat Tw2c_init = pKFsecond->GetPose().inv();
    cv::Mat Tdel_2to1 = Tw2c_init.inv() * Tw1c_init; //delta to move from cam2 to cam 1 by right multiplication
    cv::Mat baseline_arb = Tdel_2to1.rowRange(0,3).col(3);
    double baseline_arb_norm = norm(baseline_arb);
      //transform from SLAM camera to imaging camera - assumes current SLAM and Imaging frames are synchronous- could refine this
    cv::Mat Twc_slam_cur = F_ref.mTcw.inv();
    cv::Mat Twc_img_cur = Twc_slam_cur * F_ref_T;
    pKFsecond->SetPose( Twc_img_cur.inv() );
    second_frame.SetPose(Twc_img_cur.inv());
    std::cout << "map transform, new position for KFsecond: " <<  pKFsecond->GetPose() << std::endl;

    cv::Mat vint;
    if(trajectory->integrateVelocity(pKFsecond->mTimeStamp, pKFfirst->mTimeStamp,vint) == 0) { //succesfully determine motion between slam cam over mono init time

        cv::Mat Tv = vint.inv();

        //use t_v (properly rotated into image camera ref) and rotation compotent of Tdel_2to1 to determine pose of pKFini
        cv::Mat Twc_slam_ini = Twc_slam_cur * Tv;
        cv::Mat Twc_img_ini  = Twc_slam_ini * F_ref_T;
        cv::Mat Tv_img_inv   = Twc_img_cur.inv() * Twc_img_ini;  // Tv_img_inv goes from the velocity based position of the imaging camera at the current from to the initial frame
        cv::Mat tv_img_inv   = Tv_img_inv.rowRange(0,3).col(3);
        tv_img_inv.copyTo( Tdel_2to1.rowRange(0,3).col(3) ); //this step isn't really necessary could just copy position from Twc_img_ini to temp vector and then plug it back in after rotation but this is conceptually what's happening
//        fout << "trans: " << tv_img_inv.at<float>(0,0) << " " << tv_img_inv.at<float>(1,0) << " " << tv_img_inv.at<float>(2,0)   << "\t";
        Twc_img_ini = Twc_img_cur * Tdel_2to1;
        pKFfirst->SetPose( Twc_img_ini.inv() );
        std::cout << "map transform, new position for KFfirst: " <<  pKFfirst->GetPose() << std::endl;

        //correct mappoints using new, properly scaled mono-baseline and world positions
        double baseline_metric_norm = norm(tv_img_inv);
        scale = baseline_metric_norm/baseline_arb_norm;
//        std::cout << "finished seting init keyframe poses w/ metric baseline" <<std::endl;
        std::cout << "transforming map, scale: " << scale << std::endl;
    } else {
        return -1;
    }
    
    //similarity transform mappoints
    for(auto it = mpts.begin(); it != mpts.end(); ++it){
        MapPoint* pMP = *it;
        cv::Mat pos =  pMP->GetWorldPos();
        pos = scale*pos;
        
        cv::Mat pos_h(4,1, CV_32F); //homogeneous point
        pos_h.at<float>(0,0)= pos.at<float>(0,0);
        pos_h.at<float>(1,0)= pos.at<float>(1,0);
        pos_h.at<float>(2,0)= pos.at<float>(2,0);
        pos_h.at<float>(3,0)= 1.00;
        
        cv::Mat pos_new = pKFfirst->GetPoseInverse()*pos_h;
        pMP->SetWorldPos(pos_new.rowRange(0,3));
    }
        
    return 0;
        
}

int MonoInitializer::addToMap(Map* pMap){
  //  pKFfirst->setMap(pMap);
  //  pKFsecond->setMap(pMap);
    std::cout << "adding to Map KeyFrame: " << pKFfirst->mnId <<std::endl;
    pMap->AddKeyFrame(pKFfirst);

    std::cout << "adding to Map KeyFrame: " << pKFsecond->mnId <<std::endl;
    pMap->AddKeyFrame(pKFsecond);
    
    //mappoint
    for(auto it = mpts.begin(); it != mpts.end(); ++it){
        MapPoint* pMP = *it;
        int idx1 = mpts_to_keypts[pMP].first;
        int idx2 = mpts_to_keypts[pMP].second;
        pMap->AddMapPoint(pMP, pKFfirst, idx1);
        pMap->addAssociation(pKFsecond, idx2, pMP, true);
        pMP->SetProtection(5, pKFsecond->mnId); //protect from culling for 5 new keyframes
    }
    
    return 0;
    
}


Frame MonoInitializer::getInitializedFrame(){
    if(!init_success || !map_created){
        std::cout << "WARNING: MonoInitialization not Finalized, Frame not Complete!!" << std::endl;
    }
    return second_frame;
}

KeyFrame* MonoInitializer::getCurrentKF(){
    return pKFsecond;
}

void MonoInitializer::clear(){
    has_data = false; //inidicates whether first frame has been set
    init_success = false;
    
    if(mpInitializer){
        delete mpInitializer; 
        mpInitializer = static_cast<MonoEstimator*>(nullptr);
    }

 //   fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
    
    first_frame = Frame();
    second_frame = Frame();

        //resulting initial map (two keyframes + mappoints)
    pKFfirst = nullptr;
    pKFsecond = nullptr;
    mpts.clear(); 
    map_created = false;
    scale = 1.0000; // map scale 
    mpts_to_keypts.clear();  

    //match data between frames
    mvIniMatches.clear();
    mvbPrevMatched.clear();
    mvIniP3D.clear();

    Initializer::clear();

}

    int MonoInitializer::transformMapSE3(cv::Mat &Twc_SE3) {
    //NOT TESTED YET !!!!!
        cv::Mat KF1_pose_cw = pKFfirst->GetPose(); //in world to camera convention
        KF1_pose_cw =  KF1_pose_cw * Twc_SE3.inv();
        pKFfirst->SetPose(KF1_pose_cw);
        first_frame.SetPose(KF1_pose_cw);

        cv::Mat KF2_pose_cw = pKFsecond->GetPose(); //in world to camera convention
        KF2_pose_cw =  KF2_pose_cw * Twc_SE3.inv();
        pKFsecond->SetPose(KF2_pose_cw);
        second_frame.SetPose(KF2_pose_cw);

        for(auto it = mpts.begin(); it != mpts.end(); ++it){
            MapPoint* pMP = *it;
            pMP->applyTransform(Twc_SE3);
        }
        return 0;
    }

} //end namespace
