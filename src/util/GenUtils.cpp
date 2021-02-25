#include <GenUtils.h>
#include <Camera.h>
#include <Converter.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM2{

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
    double ang_v = 2*acos(w_v);
    double ang_v_scaled = ang_v * scale;
    double w = cos(ang_v_scaled/2);
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

} //end namespace
