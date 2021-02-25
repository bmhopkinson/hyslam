#ifndef INITIALIZER_H_
#define INITIALIZER_H_

#include <Frame.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <Trajectory.h>

namespace ORB_SLAM2{
class InitializerData
{
    //  std::vector<int> mvIniLastMatches;  //doesn't seem to be used
public:
    void clear();

    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;
};

class Initializer{
public:
    Initializer(){};
    virtual ~Initializer(){};

    virtual int initialize(Frame &frame) = 0;
    virtual int createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints) = 0;
    virtual int transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T) = 0;
    virtual Frame getInitializedFrame() = 0;
    virtual KeyFrame* getCurrentKF() = 0;
    virtual int addToMap(Map* pMap) =0 ;
    virtual void clear();
    InitializerData getInitializerData(){return init_data;}
protected:
    InitializerData init_data;


};

} //end namespace
#endif
