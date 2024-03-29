#include <TrackingState.h>
#include <Camera.h>

namespace HYSLAM {
TrackingState::TrackingState(std::ofstream &log, MainThreadsStatus* thread_status_):
thread_status(thread_status_)
{
    pftracking = &log;
}

std::vector<KeyFrame*> TrackingState::newKeyFrame(Frame &current_frame, Map *pMap, unsigned int  last_keyframe_id, bool force){
    std::vector<KeyFrame*> newKFs;
    if(needNewKeyFrame(current_frame, pMap, last_keyframe_id, force)){
        newKFs = createNewKeyFrame(current_frame, pMap);
    }
    return newKFs;
}


std::vector<KeyFrame*>  TrackingState::createNewKeyFrame(Frame &current_frame, Map* pMap){
    std::vector<KeyFrame*> newKFs;
    Camera camera = current_frame.getCamera();

    KeyFrame* pKFnew = new KeyFrame(current_frame);
    newKFs.push_back(pKFnew);

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
        const FeatureViews views = current_frame.getViews();
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

    return newKFs;
}

}