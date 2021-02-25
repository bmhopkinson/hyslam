#include <Triangulator.h>

namespace ORB_SLAM2 {

    bool Triangulator::DirectLinearTriangulation(cv::Mat uv1, cv::Mat uv2, cv::Mat P1, cv::Mat P2, cv::Mat &x3D) {
        cv::Mat A(4, 4, CV_32F);
        A.row(0) = uv1.at<float>(0) * P1.row(2) - P1.row(0);
        A.row(1) = uv1.at<float>(1) * P1.row(2) - P1.row(1);
        A.row(2) = uv2.at<float>(0) * P2.row(2) - P2.row(0);
        A.row(3) = uv2.at<float>(1) * P2.row(2) - P2.row(1);

        cv::Mat w, u, vt;
        cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        x3D = vt.row(3).t();

        if (x3D.at<float>(3) == 0)
            return false;

        // Euclidean coordinates
        x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
        return true;
    }

}//end namespace