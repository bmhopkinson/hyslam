#include <GenUtils.h>
#include <Camera.h>
#include <Converter.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sys/stat.h>
#include <thread>

namespace HYSLAM{

int GenUtils::Epipole(KeyFrame* pKF1, KeyFrame* pKF2, float& ex, float& ey){
    cv::Mat uv_ur;
    pKF2->ProjectLandMark(pKF1->GetCameraCenter(), uv_ur);
    ex = uv_ur.at<float>(0,0);
    ey = uv_ur.at<float>(1,0);
    return 0;

}

void GenUtils::ScaleVelocity(cv::Mat mVel, double vel_dt, double target_dt, cv::Mat &mVel_scaled) {
    cv::Mat mVwc = cv::Mat::eye(4,4,CV_32F);
    double scale = std::abs(target_dt/vel_dt);
    cv::Mat mVelwc = mVel.inv();
    cv::Mat t_v = mVelwc.rowRange(0,3).col(3);
    cv::Mat t_v_scaled = t_v * scale;  //dist of slam camera travelled over interval between mono imaging frames

    //determine rotation undergone by imaging camera, based on rotational velocity of slam camera
    cv::Mat Rbase = mVelwc.rowRange(0,3).colRange(0,3);
    Eigen::Matrix<double,3,3> Rbase_E = Converter::toMatrix3d(Rbase);
    Eigen::Quaternion<double> qbase(Rbase_E);  //easiest to convert velocity to net rotation using quaternions
    double w_v = qbase.w();
    double ang_v_scaled;
    double w;
    if(w_v>1.0000000000000){ //can get extremely small rotations and numerical problems result with w_v exceeding 1 and then everything goes haywire
        w = 1.00000000000000000;
        ang_v_scaled = 0.000000000000000000000000000;
        //std::cout << "ScaleVelocity caught w_v >1.0 error" << std::endl;
    } else {
        double ang_v = 2*acos(w_v);
        ang_v_scaled = ang_v * scale;
        w = cos(ang_v_scaled/2);
    }
    //std::cout << std::fixed << std::setprecision(9) << "w_v: " << w_v << ", ang_v_scaled: " << ang_v_scaled << ", w: " << w << std::endl;
    Eigen::Vector3d axis;
    axis << qbase.x() , qbase.y() , qbase.z() ;
    axis.normalize();
    axis = sin(ang_v_scaled/2)*axis;
    Eigen::Quaternion<double> q_v(w, axis(0), axis(1), axis(2));
    Eigen::Matrix<double,3,3> Rv_E = q_v.toRotationMatrix();

    //assemble final matrix
    cv::Mat Rv = Converter::toCvMat(Rv_E);
    cv::Mat Tv = cv::Mat::eye(4,4,CV_32F);
    Rv.copyTo( mVwc.rowRange(0,3).colRange(0,3) );
    t_v_scaled.copyTo(mVwc.rowRange(0,3).col(3) );

    if((target_dt/vel_dt) < 0.0){
        mVwc = mVwc.inv();  //assume velocity was computed from forward camera motion so need to invert for going back in time
    }

    mVel_scaled = mVwc.clone();
    mVel_scaled = mVel_scaled.inv();

}

cv::Mat GenUtils::ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2){
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat K1 = pKF1->getCamera().K.clone();
    const cv::Mat K2 = pKF2->getCamera().K.clone();


    return K1.t().inv()*t12x*R12*K2.inv();
}

cv::Mat GenUtils::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}


bool GenUtils::PointHasPositiveDepth(cv::Mat ProjectionMatrix, cv::Mat point){
    if(point.rows < point.cols){
        point = point.t();
    }
    if(point.rows < 3){
        std::cout << "PointHasPositiveDepth point dimensions too small" <<std::endl;
        return false;
    }
    if(point.rows == 3){//make homogeneous
        cv::Mat one = cv::Mat::ones(1,1,CV_32F);
        point.push_back(one);
    }
    //cv::Mat
    float z = ProjectionMatrix.row(2).dot(point.t());
   // std::cout << "z_alt: " << z <<std::endl;
    return z >= 0.000;

}


int GenUtils::mkdirRecursive(const char* pathname, mode_t mode) {
    std::string pathstring(pathname);
    std::string pathtemp;

    int ret = 0;
    int searched = 0;
    do {
        searched = pathstring.find('/', searched+1);
        pathtemp.assign(pathstring, 0, searched);
        ret = mkdir(pathtemp.c_str(), mode);
    } while (searched != std::string::npos);

    return ret;
}

void GenUtils::pauseUntilReady(KeyFrame *pKF) {
    bool paused = false;
    while(!pKF->isReady()){
        std::this_thread::sleep_for(std::chrono::microseconds(100));
       // std::cout << "!!!!!!!!!!!!!!!!paused for keyframe to be ready!!!!!!!!!!!!!!!!!!: " << pKF->mnId << std::endl;
        paused = true;
    }
    if(paused){
    //    std::cout <<"!!!!!!!!!!!!!!!!paused for keyframe to be ready!!!!!!!!!!!!!!!!!!: " << pKF->mnId << std::endl;
    }
}

void GenUtils::sparsifyMap(Map *pMap, double overlap_criterion) {
// should make this more general - currently considers successive keyframes and determines if 'overlap_criterion' fraction of landmarks matched in KF1 are visible in KF2. if so KF2 is culled.
// repeat until a KF is not culled then move on to next KF
    std::vector<KeyFrame*> allKFs =  pMap->getAllKeyFramesIncludeSubmaps();
    sort(allKFs.begin(), allKFs.end(), KeyFrame::lId);

    auto it = allKFs.begin();
    KeyFrame* pKFcur = *it;
    KeyFrame* pKFtarget = *(++it);

    while(it != allKFs.end()){

        std::vector<MapPoint*> lms_cur = pKFcur->getAssociatedLandMarks();
        int nvis = 0;
        for(auto it2 = lms_cur.begin(); it2 != lms_cur.end(); ++it2){
            if(pKFtarget->isLandMarkVisible(*it2)){
                ++nvis;
            }
        }

        double frac_visible = static_cast<double>(nvis)/static_cast<double>(lms_cur.size());
        if(frac_visible > overlap_criterion){
            pMap->SetBadKeyFrame(pKFtarget);

            pKFtarget = *(++it);
        } else {
            pKFcur = pKFtarget;
            pKFtarget= *(++it);
        }
    }

}

} //end namespace
