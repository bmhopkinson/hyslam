
#include <MapPointFactory.h>
#include <FeatureViews.h>
#include <FeatureDescriptor.h>
#include "ORBSLAM_datastructs.h"

namespace HYSLAM{

MapPointFactory::MapPointFactory(){}

MapPoint* MapPointFactory::construct( const cv::Mat &Pos, KeyFrame* pKF, int idx){
    
     MapPoint* pMP = new MapPoint(Pos); //goal is to elminate passing KF here     
     pMP->mnFirstKFid = pKF->mnId;
     return pMP;
}


MapPoint* MapPointFactory::construct(const cv::Mat &Pos, Frame* frame, int idx){
    MapPoint* pMP = new MapPoint(Pos);//goal is to elminate passing Frame here
    
   // pMP->mnFirstFrame = frame->mnId;
    
    cv::Mat Ow = frame->GetCameraCenter();
    cv::Mat vec_pc = Pos - Ow;
    const float dist = cv::norm(vec_pc);
    cv::Mat normal = vec_pc/dist;
    
    std::cout << " MPFac: dist, norm: " << dist << "\t" << normal << std::endl;
    
    pMP->setNormal(normal);

    
    const FeatureViews views = frame->getViews();
    FeatureExtractorSettings orb_params = views.getOrbParams();

    float max_dist = 2.0 * dist;
    float min_dist = 0.5 * dist;
   // std::cout << " MPFac: min_dist, max_dist " << min_dist << "\t" << max_dist <<std::endl;
    pMP->setMinDistanceInvariance(min_dist);
    pMP->setMaxDistanceInvariance(max_dist);

    FeatureDescriptor descriptor = views.descriptor(idx);
  //  std::cout << " MPFac: descriptor: " << descriptor << std::endl;
    pMP->setDescriptor(descriptor);
    
    return pMP;
}

}//end namepsace
