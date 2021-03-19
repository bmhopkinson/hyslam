#include <ORBstereomatcher.h>
#include <FeatureMatcher.h>


namespace HYSLAM{

ORBstereomatcher::ORBstereomatcher(FeatureExtractor* mpORBextractorLeft_, FeatureExtractor* mpORBextractorRight_, FeatureViews views,
                                   DescriptorDistance* descriptor_distance_, Camera cam_data, FeatureMatcherSettings settings){
  mpORBextractorLeft  = mpORBextractorLeft_;
  mpORBextractorRight = mpORBextractorRight_;
  mvKeys = views.getKeys();
  mvKeysRight = views.getKeysR();
  mDescriptors = views.getDescriptors();
  mDescriptorsRight = views.getDescriptorsR();
  descriptor_distance = descriptor_distance_;

  TH_HIGH = settings.TH_HIGH;
  TH_LOW = settings.TH_LOW;

  N = mvKeys.size();

  orb_params = views.orbParams();

  float fx = cam_data.fx();
  mbf = cam_data.mbf;
  mb = mbf/fx;
}

void ORBstereomatcher::getData(std::vector<float> &mvuRight_, std::vector<float> &mvDepth_){
  mvuRight_ = mvuRight;
  mvDepth_ = mvDepth;
}

void ORBstereomatcher::getData(FeatureViews &views){
    views.setuRs(mvuRight);
    views.setDepths(mvDepth);
}

void ORBstereomatcher::computeStereoMatches()
{
    mvuRight = std::vector<float>(N,-1.0f);
    mvDepth = std::vector<float>(N,-1.0f);

    const float dist_threshold = (TH_HIGH + TH_LOW) / 2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    std::vector<std::vector<size_t> > vRowIndices(nRows,std::vector<size_t>());

    for(int i=0; i<nRows; i++)
      vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
      const cv::KeyPoint &kp = mvKeysRight[iR];
      const float &kpY = kp.pt.y;
      const float r = 2.0f* orb_params.mvScaleFactors[mvKeysRight[iR].octave];
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
    std::vector<std::pair<int, int> > vDistIdx;
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
                //  std::cout << "StereoMatcher, dist:  " << dist << std::endl;

                  if(dist<bestDist)
                  {
                      bestDist = dist;
                      bestIdxR = iR;
                  }
              }
          }
         // std::cout << "StereoMatcher, bestDist:  " << bestDist << std::endl;

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
                  vDistIdx.push_back(std::pair<int,int>(bestDist,iL));
              }
              /*
              // coordinates in image pyramid at keypoint scale
              const float uR0 = mvKeysRight[bestIdxR].pt.x;
              const float scaleFactor = orb_params.mvInvScaleFactors[kpL.octave];
              const float scaleduL = round(kpL.pt.x*scaleFactor);
              const float scaledvL = round(kpL.pt.y*scaleFactor);
              const float scaleduR0 = round(uR0*scaleFactor);

              // sliding window search
              const int w = 5;
              cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
              IL.convertTo(IL,CV_32F);
              IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

              float bestDist = std::numeric_limits<float>::max();
              int bestincR = 0;
              const int L = 5;
              std::vector<float> vDists;
              vDists.resize(2*L+1);

              const float iniu = scaleduR0+L-w;
              const float endu = scaleduR0+L+w+1;
              if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                  continue;

              for(int incR=-L; incR<=+L; incR++)
              {
                  cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                  IR.convertTo(IR,CV_32F);
                  IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                  float dist = cv::norm(IL,IR,cv::NORM_L1);
                  if(dist<bestDist)
                  {
                      bestDist =  dist;
                      bestincR = incR;
                  }

                  vDists[L+incR] = dist;
              }

              if(bestincR==-L || bestincR==L)
                  continue;

              // Sub-pixel match (Parabola fitting)
              const float dist1 = vDists[L+bestincR-1];
              const float dist2 = vDists[L+bestincR];
              const float dist3 = vDists[L+bestincR+1];

              const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

              if(deltaR<-1 || deltaR>1)
                  continue;

              // Re-scaled coordinate
              float bestuR = orb_params.mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

              float disparity = (uL-bestuR);

              if(disparity>=minD && disparity<maxD)
              {
                  if(disparity<=0)
                  {
                      disparity=0.01;
                      bestuR = uL-0.01;
                  }
                  mvDepth[iL]=mbf/disparity;
                  mvuRight[iL] = bestuR;
                  vDistIdx.push_back(std::pair<int,int>(bestDist,iL));
              }
               */
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
