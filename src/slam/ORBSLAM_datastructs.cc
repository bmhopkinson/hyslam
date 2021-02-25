#include <ORBSLAM_datastructs.h>

namespace ORB_SLAM2 {

Imgdata::Imgdata(std::string name_, std::string camera_, double time_stamp_) :
          name(name_), camera(camera_), time_stamp(time_stamp_){}

ORBExtractorParams::ORBExtractorParams(ORBextractor* extractor){
  setParams(extractor);
}

void ORBExtractorParams::setParams(ORBextractor* extractor){
    mnScaleLevels = extractor->GetLevels();
    mfScaleFactor = extractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = extractor->GetScaleFactors();
    mvInvScaleFactors = extractor->GetInverseScaleFactors();
    mvLevelSigma2 = extractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = extractor->GetInverseScaleSigmaSquares();
}

} //close namespace
