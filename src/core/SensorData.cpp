#include <SensorData.h>

namespace ORB_SLAM2{
    std::vector<double> SensorData::getGPSerr() const {
        std::vector<double> err = {gps.ll_err.first, gps.ll_err.second, gps.alt_err};
        return err;
    }


}//end namespace