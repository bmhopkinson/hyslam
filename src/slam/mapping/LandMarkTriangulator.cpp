#include <LandMarkTriangulator.h>
#include <Camera.h>
#include <FeatureViews.h>
#include <ORBExtractorParams.h>
#include <GenUtils.h>
#include <FeatureMatcher.h>

namespace HYSLAM {
LandMarkTriangulator::LandMarkTriangulator(KeyFrame *pKF_, Map *pMap_, std::list<MapPoint *> *new_mpts_,LandMarkTriangulatorParameters params_, std::ofstream &log_ ) :
    pKF(pKF_), pMap(pMap_), new_mpts(new_mpts_), params(params_) {
log = &log_;
}

void LandMarkTriangulator::run() {
    // Retrieve neighbor keyframes in covisibility graph
    const Camera camera_KF1 = pKF->getCamera();
    
    bool is_mono = camera_KF1.sensor == 0;
    int nn = params.N_neighborKFs_stereo;
    if(is_mono) //monocular
        nn = params.N_neighborKFs_mono;
    const std::vector<KeyFrame*> vpNeighKFs = pMap->getKeyFrameDB()->GetBestCovisibilityKeyFrames(pKF, nn);

    const FeatureViews& KFcur_views = pKF->getViews();
    ORBExtractorParams orb_params_KFcur = KFcur_views.orbParams();
    const Camera camera_KFcur = pKF->getCamera();

    FeatureMatcher matcher(params.match_nnratio, false);

    cv::Mat Ow1 = pKF->GetCameraCenter();

    const float ratioFactor = params.ratio_factor *orb_params_KFcur.mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && abort_requested) {
            std::cout << "aborting LandMarkTriangulation after n keyframes: " << i << std::endl;
            has_finished = true;
            return;
        }

        KeyFrame* pKF2 = vpNeighKFs[i];
        const FeatureViews& KF2views = pKF2->getViews();
        ORBExtractorParams orb_params_KF2 = KF2views.orbParams();
        const Camera camera_KF2 = pKF2->getCamera();

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!is_mono)
        {
            if(baseline < camera_KF2.mb())
                continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth < params.min_baseline_depth_ratio) {
                continue;
            }
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = GenUtils::ComputeF12(pKF,pKF2);

        std::vector<std::pair<size_t,size_t> > vMatchedIndices;
        int nmatches_search = matcher.SearchForTriangulation(pKF,pKF2,F12,vMatchedIndices,false);
   //     std::cout << "LMTriangulator: pKF1 " << pKF->mnId << " pKF2: " << pKF2->mnId << " ; matches: " << nmatches_search << std::endl;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint kp1 = KFcur_views.keypt(idx1);
            const float kp1_ur = KFcur_views.uR(idx1);
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint kp2 = KF2views.keypt(idx2);
            const float kp2_ur =  KF2views.uR(idx2);
            bool bStereo2 = kp2_ur>=0;

            //Check parallax between rays
            cv::Mat ray1 = pKF->BackProjectLandMarkView(idx1, 1.00)  - pKF->GetCameraCenter(); //rays with same origin but rotatedd and backprojected correctly
            cv::Mat ray2 = pKF2->BackProjectLandMarkView(idx2, 1.00) - pKF2->GetCameraCenter();
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

           // std::cout << "lm triang cos parallax, std: " << cosParallaxRays << " , alt: " << cosParallaxRays_alt << std::endl;

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1){
                float mb_KFcur = camera_KFcur.mb();
                cosParallaxStereo1 = cos(2*atan2( mb_KFcur/2, KFcur_views.depth(idx1) ));
            }
            else if(bStereo2){
                float mb_KF2 = camera_KF2.mb();
                cosParallaxStereo2 = cos(2*atan2(mb_KF2/2, KF2views.depth(idx2) ));
            }

            cosParallaxStereo = std::min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat P1 = pKF->getCameraMatrix();
            cv::Mat P2 = pKF2->getCameraMatrix();

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                cv::Mat uv1 = (cv::Mat_<float>(2,1) << kp1.pt.x, kp1.pt.y);
                cv::Mat uv2 = (cv::Mat_<float>(2,1) << kp2.pt.x, kp2.pt.y);

               if(!triangulator.DirectLinearTriangulation(uv1, uv2, P1, P2, x3D)){
                   continue;
               }

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = pKF->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else {
                continue; //No stereo and very low parallax
            }

            //ensure triangulated point is in front of both cameras
            bool z1_positive = GenUtils::PointHasPositiveDepth(P1, x3D);
            bool z2_positive = GenUtils::PointHasPositiveDepth(P2, x3D);
            if( !(z1_positive && z2_positive) ){
                continue;
            }

            //Check reprojection error
            const float &sigmaSquare1 = orb_params_KFcur.mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = orb_params_KF2.mvLevelSigma2[kp2.octave];
            float err_thresh1 = params.error_factor_mono * sigmaSquare1;
            if(bStereo1){
                err_thresh1 = params.error_factor_stereo * sigmaSquare1;
            }

            float err_thresh2 = params.error_factor_mono * sigmaSquare2;
            if(bStereo2){
                err_thresh2 = params.error_factor_stereo * sigmaSquare2;
            }

            float err1 = pKF->ReprojectionError(x3D, idx1 );
            float err2 = pKF2->ReprojectionError(x3D, idx2 );
            bool pass = true;
            if( (err1 > err_thresh1) || (err2 > err_thresh2) ){
                pass = false;
                continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = orb_params_KFcur.mvScaleFactors[kp1.octave]/orb_params_KF2.mvScaleFactors[kp2.octave];
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // triangulation of new point succeeded  - add to map and associate
            MapPoint* pMP = pMap->newMapPoint(x3D, pKF, idx1);
            pMap->addAssociation(pKF2, idx2, pMP, true);

            new_mpts->push_back(pMP);

            nnew++;
        }
    }
    *log << "new_triang_mpts: " << nnew <<"\t";
    has_finished = true;

}

} //end namespace
