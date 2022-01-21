#include <KeyFrameCuller.h>
#include <KeyFrameDB.h>
#include <Camera.h>
#include <FeatureViews.h>

#include <vector>
#include <map>

namespace HYSLAM{

KeyFrameCuller::KeyFrameCuller(KeyFrame *pKF_, Map *pMap_, KeyFrameCullerParameters params_, std::ofstream &log_) :
pKF(pKF_), pMap(pMap_), params(params_)
{
    log = &log_;
}

KeyFrameCuller::~KeyFrameCuller(){
   // std::cout << "destroying KFculler for KF: " << pKF->mnId << std::endl; //for testing
}

void KeyFrameCuller::run(){
    const Camera camera = pKF->getCamera();
    bool is_mono = camera.sensor == 0;

    std::vector<KeyFrame*> vpLocalKeyFrames = pMap->getVectorCovisibleKeyFrames(pKF);

    for(std::vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        if(pKFi->mnId==0 )//|| pKFi->mnId == pKF->mnId)  //second condition really shouldn't happen but for some reason it is.
            continue;
        const std::vector<MapPoint*> vpMapPoints = pKFi->GetMapPointMatches();
        const FeatureViews& KFviews = pKFi->getViews();

        const int thObs= params.LMobservations_thresh;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {

                    if(!is_mono)
                    {
                        float depth_pt = KFviews.depth(i);
                        if( depth_pt > pKFi->mThDepth ||depth_pt <0) //if the mappoint is far away (>mThDepth) and so expected to be viewed in many keyframes, ignore. also if depth is not valid ignore.
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = KFviews.keypt(i).octave;
                        const std::map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(std::map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi2 = mit->first;
                            if(pKFi2==pKFi)
                                continue;

                            const FeatureViews& KF2views = pKFi2->getViews();
                            const int &scaleLeveli = KF2views.keypt(mit->second).octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }

        }

        if(nRedundantObservations > params.frac_redundant * nMPs){
            std::cout << "cull KF: " << pKFi->mnId << std::endl;
            *log << "cull KF: " << pKFi->mnId << "\t";
            pMap->SetBadKeyFrame(pKFi);
        }
    }
    has_finished = true;
}

}//end namespace