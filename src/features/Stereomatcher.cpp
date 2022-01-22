#include <Stereomatcher.h>
#include <FeatureMatcher.h>


namespace HYSLAM{

Stereomatcher::Stereomatcher( FeatureViews views, Camera cam_data, FeatureMatcherSettings settings) : camera(cam_data)
                                   {
  mvKeys = views.getKeys();
  mvKeysRight = views.getKeysR();
  mDescriptors = views.getDescriptors();
  mDescriptorsRight = views.getDescriptorsR();

  TH_HIGH = settings.TH_HIGH;
  TH_LOW = settings.TH_LOW;

  N = mvKeys.size();

  orb_params = views.orbParams();

  float fx = cam_data.fx();
  mbf = cam_data.mbf;
  mb = mbf/fx;
}

void Stereomatcher::getData(std::vector<float> &mvuRight_, std::vector<float> &mvDepth_){
  mvuRight_ = mvuRight;
  mvDepth_ = mvDepth;
}

void Stereomatcher::getData(FeatureViews &views){
    views.setuRs(mvuRight);
    views.setDepths(mvDepth);
}

void Stereomatcher::computeStereoMatches()
{
    mvuRight = std::vector<float>(N,-1.0f);
    mvDepth = std::vector<float>(N,-1.0f);

    const float dist_threshold = (TH_HIGH + TH_LOW) / 2;

    const int nRows = camera.mnMaxY;

    //Assign keypoints to row table
    std::vector<std::vector<size_t> > vRowIndices(nRows,std::vector<size_t>());

    for(int i=0; i<nRows; i++)
      vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
      const cv::KeyPoint &kp = mvKeysRight[iR];
      const float &kpY = kp.pt.y;
      const float r = 2.0f*  kp.size/orb_params.size_ref;
      const int maxr = ceil(kpY+r);
      const int minr = floor(kpY-r);

      for(int yi=minr;yi<=maxr;yi++)
        vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    std::vector<std::pair<float, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const std::vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

          float bestDist = TH_HIGH;
          size_t bestIdxR = 0;

          const FeatureDescriptor &dL = mDescriptors[iL];

          // Compare descriptor to right keypoints
          for(size_t iC=0; iC<vCandidates.size(); iC++)
          {
              const size_t iR = vCandidates[iC];
              const cv::KeyPoint &kpR = mvKeysRight[iR];

              if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                  continue;

              const float &uR = kpR.pt.x;

              if(uR>=minU && uR<=maxU)
              {
                  const FeatureDescriptor &dR = mDescriptorsRight[iR];
                  const float dist = dL.distance(dR);
                 // std::cout << "StereoMatcher, dist:  " << dist << std::endl;

                  if(dist<bestDist)
                  {
                      bestDist = dist;
                      bestIdxR = iR;
                  }
              }
          }

          // Subpixel match by correlation
          if(bestDist < dist_threshold)
          {
              float uR0 = mvKeysRight[bestIdxR].pt.x;
              float disparity = (uL-uR0);

              if(disparity>=minD && disparity<maxD)
              {
                  if(disparity<=0)
                  {
                      disparity=0.01;
                      uR0 = uL-0.01;
                  }
                  mvDepth[iL]=mbf/disparity;
                  mvuRight[iL] = uR0;
                  vDistIdx.push_back(std::pair<float,int>(bestDist,iL));
              }
          }

      }
      sort(vDistIdx.begin(),vDistIdx.end());
      const float median = vDistIdx[vDistIdx.size()/2].first;
      const float thDist = 1.5f*1.4f*median;

      for(int i=vDistIdx.size()-1;i>=0;i--)
      {
          if(vDistIdx[i].first<thDist)
              break;
          else
          {
              mvuRight[vDistIdx[i].second]=-1;
              mvDepth[vDistIdx[i].second]=-1;
          }
      }
  }

}//close namespace
