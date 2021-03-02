
#include <LandMarkCuller.h>
#include <Camera.h>

namespace HYSLAM{
LandMarkCuller::LandMarkCuller(KeyFrame* pKF_, Map* pMap_, std::list<MapPoint*>* new_mpts_, LandMarkCullerParameters params_, std::ofstream &log_) :
pKF(pKF_), pMap(pMap_), new_mpts(new_mpts_), params(params_)
{
    log = &log_;
}

void LandMarkCuller::run(){
    // Check Recent Added MapPoints
    std::list<MapPoint*>::iterator lit = new_mpts->begin();
    const unsigned long int nCurrentKFid = pKF->mnId;

    Camera camera = pKF->getCamera();
    int nThObs;
    if(camera.sensor == 0)  //monocular
        nThObs = params.min_lm_observations_mono;
    else
        nThObs = params.min_lm_observations_stereo;
    const int cnThObs = nThObs;

    int n_culled = 0;
    while(lit!=new_mpts->end())
    {
        MapPoint* pMP = *lit;
        if(pMP->Protected())
        {
            //  std::cout << "lowering mp protection: " << pMP->mnId << std::endl;
            pMP->LowerProtection(nCurrentKFid);
            lit++;
            continue;
        }
        else if(pMP->isBad())
        {
            lit = new_mpts->erase(lit);
        }

        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>= params.KF_grace_period && pMP->Observations()<=cnThObs)
        {
           // std::cout << "deleting map_pt due to low observations: " << pMP->mnId <<std::endl;
            ++n_culled;
            pMap->eraseMapPoint(pMP);
            lit = new_mpts->erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid) >= params.KF_grace_period+1)
            lit = new_mpts->erase(lit);
        else
            lit++;
    }
    *log << "n_culled: " << n_culled << "\t";
    has_finished = true;
}
}
