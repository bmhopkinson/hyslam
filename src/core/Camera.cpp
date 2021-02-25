#include <Camera.h>


namespace ORB_SLAM2{
float PI  = 3.14159265;

int Camera::loadData(const cv::FileNode &data_node){
    camName = data_node.name();
    sensor = data_node["sensor"];
    std::cout << camName << "is sensor type: " << sensor << std::endl;

    cv::FileNode trans = data_node["transformation"];
    float tx = trans["x"];
    float ty = trans["y"];
    float tz = trans["z"];
    float r_z = trans["roll"];
    float r_x = trans["pitch"];
    float r_y = trans["yaw"];
    cv::Mat Rz = cv::Mat::eye(3,3,CV_32F);
    Rz.at<float>(0,0) = cos(r_z*PI/180);
    Rz.at<float>(0,1) = -1*sin(r_z*PI/180);
    Rz.at<float>(1,0) = sin(r_z*PI/180);
    Rz.at<float>(1,1) = cos(r_z*PI/180);

    cv::Mat Rx = cv::Mat::eye(3,3,CV_32F);
    Rx.at<float>(1,1) = cos(r_x*PI/180);
    Rx.at<float>(1,2) = -1*sin(r_x*PI/180);
    Rx.at<float>(2,1) = sin(r_x*PI/180);
    Rx.at<float>(2,2) = cos(r_x*PI/180);

    cv::Mat Ry = cv::Mat::eye(3,3,CV_32F);
    Ry.at<float>(0,0) = cos(r_y*PI/180);
    Ry.at<float>(0,2) = sin(r_y*PI/180);
    Ry.at<float>(2,0) = -1*sin(r_y*PI/180);
    Ry.at<float>(2,2) = cos(r_y*PI/180);

    cv::Mat R = cv::Mat::eye(3,3,CV_32F);
    R = Ry*Rx*Rz;

    cv::Mat Rt = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Rt.rowRange(0,3).colRange(0,3));
    Rt.at<float>(0,3) = tx;
    Rt.at<float>(1,3) = ty;
    Rt.at<float>(2,3) = tz;

    Rt.copyTo(Tcam);
    std::cout << "Rt: " << Tcam << std::endl;

    cv::FileNode calib = data_node["calibration"];
    cv::FileNode dimensions = data_node["dimensions"];
    scale = dimensions["scale"];
    float fx = calib["fx"];
    float fy = calib["fy"];
    float cx = calib["cx"];
    float cy = calib["cy"];
    fx = scale * fx;
    fy = scale * fy;
    cx = scale * cx;
    cy  = scale * cy;

    cv::Mat K_temp = cv::Mat::eye(3,3,CV_32F);
    K_temp.at<float>(0,0) = fx;
    K_temp.at<float>(1,1) = fy;
    K_temp.at<float>(0,2) = cx;
    K_temp.at<float>(1,2) = cy;
    K_temp.copyTo(K);


    cv::Mat distCoef_temp(4,1,CV_32F);
    distCoef_temp.at<float>(0) = calib["k1"];
    distCoef_temp.at<float>(1) = calib["k2"];
    distCoef_temp.at<float>(2) = calib["p1"];
    distCoef_temp.at<float>(3) = calib["p2"];
    const float k3 = calib["k3"];
    if(k3!=0)
    {
        distCoef_temp.resize(5);
        distCoef_temp.at<float>(4) = k3;
    }

    distCoef_temp.copyTo(distCoef);

    mbf = data_node["stereo_bf"];
    mbf = scale * mbf;

    thDepth = data_node["ThDepth"];
    thDepth = mbf*thDepth/fx;

    fps = data_node["fps"];

/*
    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << distCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << distCoef.at<float>(1) << std::endl;
    if(distCoef.rows==5)
        std::cout << "- k3: " << distCoef.at<float>(4) << std::endl;
    std::cout << "- p1: " << distCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << distCoef.at<float>(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;
*/
    RGB = data_node["RGB"];

    return 0;
}

bool Camera::Project(cv::Mat Pc, cv::Mat &uv) const {
  //linear camera model right now - assume distorion has been removed from images
  //update to accept more complicated camera models at some point
  // will always provide projected point in uv (can be useful), returns whether this point is valid or not
    //linear camera model right now - assume distorion has been removed from images
    //update to accept more complicated camera models at some point

    float PcZ = Pc.at<float>(2);
    float invz = 1.0f/PcZ;
    cv::Mat Pch(3,1, CV_32F); //homogenous matrix
    Pch.at<float>(0,0) =  Pc.at<float>(0) / PcZ;
    Pch.at<float>(1,0) =  Pc.at<float>(1) / PcZ;
    Pch.at<float>(2,0) =  Pc.at<float>(2) / PcZ;

    // Project in image and check it is not outside
    uv = K*Pch;  //do projection
    float u = uv.at<float>(0,0);
    float v = uv.at<float>(1,0);

    //handle stereo right image if stereo cam
    if(sensor == 1){
        float ur = u - mbf*invz;
        uv.at<float>(2,0) = ur;
    } else {
        uv.at<float>(2,0) = -1.0;
    }

    //is projection valid
    bool valid = false;
    if(PcZ>0.0f){ // is the point in front of the camera
        if(u>=mnMinX && u<=mnMaxX){  //does point fall within image bounds
            if(v>=mnMinY && v<=mnMaxY){
                valid = true;
            }
        }
    }

    return valid;

}

cv::Mat Camera::Unproject(float u, float v, float z) const {
    float x = (u - cx())* (z/fx());
    float y = (v - cy())* (z/fy());
    return(cv::Mat_<float>(3,1) << x, y, z);
}

}//close namespace
