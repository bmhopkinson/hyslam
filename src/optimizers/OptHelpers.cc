#include "OptHelpers.h"
#include <opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>

namespace ORB_SLAM2
{

void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}


cv::Mat ComputeSim3_Horn(cv::Mat &P1, cv::Mat &P2, bool FixScale, cv::Mat &R)
{
    // From OrbSLAM Sim3 Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions
    // BH, 2020/04/20 verified consistency with a Matlab implemenation
    // computes transform from P2 from to P1 frame. e.g. apply tranform to points in P2 will get them in reference from of P1.
    // returns full sim3, passes rotation matrix to R in arguments


    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix

    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

    cv::Mat mR12i;
    mR12i.create(3,3,P1.type());

    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis
    R = mR12i.clone(); //output rotation matrix
 //   cout << "Horn Rotation: " << mR12i << endl;
    // Step 5: Rotate set 2

    cv::Mat P3 = mR12i*Pr2;

    // Step 6: Scale
    float ms12i;  //scale
    if(!FixScale)
    {
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;

  //  std::cout << "Horn Scale: " << ms12i << std::endl;

    // Step 7: Translation
    cv::Mat mt12i;
    mt12i.create(1,3,P1.type());
    mt12i = O1 - ms12i*mR12i*O2;
   // cout << "Horn Translation" << mt12i << endl;

    // Step 8: Transformation

    // Step 8.1 T12
    cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    return mT12i;
    // Step 8.2 T21
    /*
    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
 */
}

cv::Mat PoseAlignmentTransform(std::vector<cv::Mat> &T1, std::vector<cv::Mat> &T2){
  //determine a rotation that best aligns poses (in world to camera convention) whose camera centers have previously been
  // aligned by a similarity transform using ComputeSim3_Horn
  // determines rotation to transform T2 so that they align with T1;
  cv::Mat P1 = PosePoints(T1);
  cv::Mat P2 = PosePoints(T2);
  bool FixScale = true;
  cv::Mat R;
  cv::Mat Thorn = ComputeSim3_Horn(P1, P2, FixScale, R);

  return Thorn.clone();

}

cv::Mat PosePoints(std::vector<cv::Mat> &Ts){
  cv::Mat P;
  for(std::vector<cv::Mat>::const_iterator vit = Ts.begin(); vit != Ts.end(); ++vit){
    cv::Mat Tcw = *vit;
    cv::Mat Twc = Tcw.inv();
    //MAY WANT TO SCALE AXES RELATIVE TO DISTANCE BETWEEN POINTS - add SCALE parameter
    cv::Mat x_ax = (cv::Mat_<float>(4,1) << 1.00, 0.00, 0.00, 1.00);  //could combine these into a single matrix
    cv::Mat y_ax = (cv::Mat_<float>(4,1) << 0.00, 1.00, 0.00, 1.00);
    cv::Mat z_ax = (cv::Mat_<float>(4,1) << 0.00, 0.00, 1.00, 1.00);
    cv::Mat org  = (cv::Mat_<float>(4,1) << 0.00, 0.00, 0.00, 1.00);
    cv::Mat px = Twc * x_ax;
    cv::Mat py = Twc * y_ax;
    cv::Mat pz = Twc * z_ax;
    cv::Mat po = Twc * org;

    P.push_back( px.rowRange(0,3).t() );
    P.push_back( py.rowRange(0,3).t() );
    P.push_back( pz.rowRange(0,3).t() );
    P.push_back( po.rowRange(0,3).t() );
  }
  cv::Mat P_t = P.t();
  return P_t.clone();

}

Eigen::Matrix<double, 3,3> Rotate_GpsError(std::vector<double> gpserr, cv::Mat &R){
    Eigen::Matrix<double, 3, 3>  merr = Eigen::MatrixXd::Zero(3,3);
    merr(0,0) = gpserr[0]; merr(1,1) = gpserr[1]; merr(2,2) = gpserr[2];  //consider each error as a column vector
    Eigen::Matrix<double, 3, 3> Reig;
    cv::cv2eigen(R, Reig);
    Eigen::Matrix<double, 3, 3> merr_r = Reig * merr;  // rotate those column vectors
    Eigen::VectorXd v(3);
    v = merr_r.rowwise().norm();   // norm of row vectors reprsents errors in new coordinate frame (at least approximately)
    v = v.cwiseInverse();  // g2o want "information" rather than error. so take 1/err
    Eigen::Matrix<double, 3,3> gpsInf = Eigen::MatrixXd::Zero(3,3);
    gpsInf(0,0) = v(0);  gpsInf(1,1) = v(1); gpsInf(2,2) = v(2);
    return gpsInf;
}

} //end ORB_SLAM2 namespace
