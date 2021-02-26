#ifndef HYSLAM_SENSORDATA_H_
#define HYSLAM_SENSORDATA_H_

#include <string>
#include <vector>

namespace HYSLAM{
    struct GpsData
    {
        std::string time;

        std::pair<double, double> latlon;  // gps derived latitude/longitude
        std::pair<double, double> noreast;  // UTM northing easting
        double alt = 0;  // altitude, meters
        std::vector<double> relpos; //relative position in meters
        std::vector<double> origin; // origin for relative position

        std::pair<double, double> ll_err = {3,3}; //latlon, noreast error, meters
        double alt_err = 10; //altitude error
        bool  valid = 0;

    };

    struct ImuData
    {
        std::vector<double> quat;   //quaternion orientation
        std::vector<double> acc;  // accleration data
        std::vector<double> angv; // angular velocity data

        std::vector<double> quat_rel;   //relative quaternion orientation
        std::vector<double> quat_origin;   //reference quaternion
        bool valid = 0;
    };

    struct DepthData
    {
        double depth = -999;
        bool valid = 0;

    };

    class SensorData{
    public:
        //getters and setters
        //time data
        void setTimeStampNS(unsigned long long ts){timestamp_ns = ts;}
        void setTimeStampRel(double ts){timestamp_rel =ts; };
        unsigned long long getTimeStampNS() const { return timestamp_rel; }
        double getTimeStampRel() const {return timestamp_rel;}

        //depth data
        void setDepth(DepthData d){depth = d;};
        double getDepth() const {return depth.depth; }
        bool isDepthValid() const {return depth.valid; }

        //GPS data
        void setGPS(GpsData g){gps = g;}
        GpsData getGPS() const {return gps; }
        std::pair<double, double> getLatLon() const {return gps.latlon; }
        std::pair<double, double> getNorEast() const {return gps.noreast; }
        std::vector<double> getGPSRel() const {return gps.relpos; }
        std::vector<double> getGPSerr() const;
        double getAlt() const {return gps.alt; }
        bool isGPSValid() const {return gps.valid; }

        //IMU data
        void setImu(ImuData idata) {imu = idata;}
        ImuData getImu() const {return imu; }
        std::vector<double> getQuat() const {return imu.quat; }
        std::vector<double> getAccel() const {return imu.acc; }
        std::vector<double> getAngVel() const {return imu.angv; }
        bool isImuValid() const {return imu.valid; }


    private:
        unsigned long long timestamp_ns;  //unique id to match to svo frames
        double timestamp_rel;

        DepthData depth;
        GpsData gps;
        ImuData imu;

    };

}//end namespace
#endif