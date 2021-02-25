#include "Tracking_datastructs.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace ORB_SLAM2;

int main(){
     MonoInitParams params;
    //T1-T5 are sucsessive camera motions in camera to world convention 
    cv::Mat T0 = (cv::Mat_<float>(4,4) << 0.0, 0.0, 0.0, 0.00,         0.0, 0.0, 0.0, 0.0,           -0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat T1 = (cv::Mat_<float>(4,4) << 0.9659, 0.0, 0.2588, 0.50,   0.0, 1.0, 0.0, 1.0,          -0.2588, 0.0, 0.9659, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat T2 = (cv::Mat_<float>(4,4) << 1.0, 0.0, 0.0, 0.0,          0.0, 0.9659, -0.2588, 0.75,   0.0, 0.2588, 0.9659,-0.5,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat T3 = (cv::Mat_<float>(4,4) << 0.9848, -0.1736, 0.0, 0.50,  0.1736,  0.9848, 0.0, 0.0,    0.0, 0.0, 1.0, 1.0,         0.0, 0.0, 0.0, 1.0 );
    cv::Mat T4 = (cv::Mat_<float>(4,4) << 1.0, 0.0, 0.0, 0.5,          0.0, 0.9962, -0.0872, -0.25,  0.0, 0.0872, 0.9962, 0.0,   0.0, 0.0, 0.0, 1.0 );
    cv::Mat T5 = (cv::Mat_<float>(4,4) << 0.9659,-0.2588, 0.0, 1.0,    0.2588, 0.9659, 0.0, -2.0,    0.0, 0.0, 1.0, 0.5,         0.0, 0.0, 0.0, 1.0 );
    
 //   std::cout << "T2: " << T2 << std::endl;
    double time0 = 0.0;    
    double time1 = 1.0;
    double time2 = 2.0;
    double time3 = 3.0;
    double time4 = 4.0;
    double time5 = 5.0;
    
    //invert T1-T5 so they are world to camera transformations as they would be in ORB SLAM.
    // load them and their respective times into a  Velocity structure (T1 first -> T5 last)
    // test Integrate Velocity
    Velocity velocities;
    velocities.addElement(T0, time0, true); //zero is a dummy - don't invert
    velocities.addElement(T1.inv(), time1, true);
    velocities.addElement(T2.inv(), time2, true);
    velocities.addElement(T3.inv(), time3, true);
    velocities.addElement(T4.inv(), time4, true);
    velocities.addElement(T5.inv(), time5, true);
    
    std::vector<cv::Mat> vels; 
    std::vector<double> times;
    velocities.timeRange(2.01, 4.00, vels, times);
    
    std::cout << "vels[0] " << vels[0] << " times[0]" << times[0] << std::endl;
    std::cout << "vels[1] " << vels[1] << " times[1]" << times[1] << std::endl;
    std::cout << "vels[2] " << vels[2] << " times[2]" << times[2] << std::endl;
    std::cout << "vels[3] " << vels[3] << " times[3]" << times[3] << std::endl;   
 //   std::cout << "vels[4] " << vels[4] << " times[4]" << times[4] << std::endl; 
        
    double t_start = 1.001;
    double t_stop = 5.00;
    cv::Mat vel_int;
    //first test:
    velocities.integrate(t_start, t_stop, vel_int);
    std::cout << "test 1: " << vel_int.inv() << std::endl;
    
    t_start = 0.5;
    t_stop = 4.5;
    //first test:
    velocities.integrate(t_start, t_stop, vel_int);
    std::cout << "test 2: " << vel_int.inv() << std::endl;
    cv::Mat Test1 = vel_int.inv();
    
    velocities.integrate(t_stop, t_start, vel_int);
    std::cout << "test 3: " << vel_int.inv() << std::endl;
    std::cout << "should be I: " << Test1 * vel_int.inv() << std::endl;
    
    
    t_start = 2.001;
    t_stop = 4.0;
    //first test:
    velocities.integrate(t_start, t_stop, vel_int);
    std::cout << "test 4: " << vel_int.inv() << std::endl;
    cv::Mat T4to2;
    T4to2 = T4.inv() * T3.inv() ;
    std::cout << "test 4 target : " << T4to2.inv() << std::endl;
    
    velocities.integrate(t_stop, t_start, vel_int);
    std::cout << "test 5: " << vel_int.inv() << std::endl;
    std::cout << "test 5 target : " << T4to2 << std::endl;
    
    cv::Mat Vscaled;
    velocities.ScaleVelocity(T1, 1.0, -0.5, Vscaled);
    std::cout << "Vscaled: " << Vscaled << std::endl;
    
    
}
