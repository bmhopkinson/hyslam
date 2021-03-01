#include <ORBSLAM_datastructs.h>

namespace HYSLAM {

Imgdata::Imgdata(std::string name_, std::string camera_, double time_stamp_) :
          name(name_), camera(camera_), time_stamp(time_stamp_){}

ORBExtractorParams::ORBExtractorParams(FeatureExtractor* extractor){
  setParams(extractor);
}

void ORBExtractorParams::setParams(FeatureExtractor* extractor){
    mnScaleLevels = extractor->GetLevels();
    mfScaleFactor = extractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = extractor->GetScaleFactors();
    mvInvScaleFactors = extractor->GetInverseScaleFactors();
    mvLevelSigma2 = extractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = extractor->GetInverseScaleSigmaSquares();
}

} //close namespace
