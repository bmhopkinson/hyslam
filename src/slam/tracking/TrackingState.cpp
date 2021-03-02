#include <TrackingState.h>
#include <Camera.h>
#include <Mapping.h>

namespace HYSLAM {
TrackingState::TrackingState(std::ofstream &log)
{
    pftracking = &log;
}

KeyFrame* TrackingState::newKeyFrame(Frame &current_frame, Map *pMap, Mapping *pLocalMapper, unsigned int  last_keyframe_id, bool force){
    KeyFrame* pKFnew = nullptr;
    if(needNewKeyFrame(current_frame, pMap, pLocalMapper, last_keyframe_id, force)){
        pKFnew = createNewKeyFrame(current_frame, pMap, pLocalMapper);
    }
    return pKFnew;
}


KeyFrame*  TrackingState::createNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper){
    KeyFrame* pKFnew = nullptr;
    Camera camera = current_frame.getCamera();
    if(!pLocalMapper->SetNotStop(true))
        return pKFnew;

    pKFnew = new KeyFrame(current_frame);
    current_frame.mpReferenceKF = pKFnew;

    (*pftracking) << pKFnew->mnId << "\t";
    if(!camera.sensor == 0) //if not monocular camera
    {
        current_frame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < thDepth.
        // If there are less than 100 close points we create the 100 closest.
        std::vector<std::pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(current_frame.N);
        const ORBViews views = current_frame.getViews();
        for(int i=0; i<current_frame.N; i++)
        {
            float z = views.depth(i);
            if(z>0)
            {
                vDepthIdx.push_back(std::make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            int new_mpts = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = current_frame.hasAssociation(i);
                if(!pMP) {
                   bCreateNew = true;
                }
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    current_frame.removeLandMarkAssociation(i);
                }

                if(bCreateNew)
                {
                    new_mpts++;
                    cv::Mat x3D = current_frame.UnprojectStereo(i);
                    MapPoint* pNewMP = pMap->newMapPoint(x3D, pKFnew, i);

                    current_frame.associateLandMark(i, pNewMP, false);
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first > camera.thDepth && nPoints>100)
                    break;
            }
            (*pftracking) << new_mpts << "\t";
        }
    }
    //std::cout << "inserting new KF into map: " << pKF->mnId << std::endl;
    pLocalMapper->InsertKeyFrame(pKFnew);
    pLocalMapper->SetNotStop(false);

    return pKFnew;
}

}