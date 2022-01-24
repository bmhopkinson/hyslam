#ifndef INITIALIZER_H_
#define INITIALIZER_H_

/*
 * abstract base class for slam initialization
 * key functionality:
 * int initialize(Frame &frame)
 *      attempts initialization procedure on frame. returns 0 if initialization is successful
 *
 * int createMap( KeyFrame* &pKF1, KeyFrame* &pKF2, std::vector<MapPoint*> mappoints)
 *      creates an initial map from the data passed in, stores new map internally not in official "Map" data structure
 *
 * int transformMap(Trajectory* trajectory, const Frame &F_ref, cv::Mat F_ref_T)
 *      transforms new initialized map to align with existing data
 *
 * int addToMap(Map* pMap)
 *      adds internal initial map data to existing map pMap;
 *
 * Frame getInitializedFrame()
 *      returns updated Frame after initialization
 */

#include <Frame.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <Trajectory.h>

namespace HYSLAM{
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
    virtual int transformMapSE3(cv::Mat &Twc_SE3) = 0;
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
