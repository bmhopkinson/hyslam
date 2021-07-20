#include <ProcessNewKeyFrame.h>
#include <MapPoint.h>
#include <thread>

namespace HYSLAM{
ProcessNewKeyFrame::ProcessNewKeyFrame(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, ProcessNewKeyFrameParameters params_):
pKF(pKF_), pMap(pMap_), new_mpts(new_mpts_), params(params_)
{}

void ProcessNewKeyFrame::run(){
    // Compute Bags of Words structures
    pKF->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const std::vector<MapPoint*> vpMapPointMatches = pKF->GetMapPointMatches();
 //   std::cout << "about to associate mappoint to new keyframe" << std::endl;
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(pKF))
                {
                    pMap->addAssociation(pKF, i, pMP, false);
                }
                else // this can only happen for new stereo points inserted byTracking
                {
                    new_mpts->push_back(pMP);  // this is not threadsafe!!!
                }
            }
        }
    }

    pMap->AddKeyFrame(pKF);
    pKF->setReady(); //keyframe now ready to be used generally
    std::cout << "mapping inserted KF into map: " << pKF->mnId << std::endl;
    has_finished = true;
}

}