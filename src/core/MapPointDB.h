#ifndef HYSLAM_MAPPOINTDB_H_
#define HYSLAM_MAPPOINTDB_H_

#include <MapPoint.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <FeatureDescriptor.h>
#include <opencv2/core/core.hpp>

#include <map>
#include <mutex>

namespace HYSLAM{
    class MapPointDBEntry{
    public:
        MapPointDBEntry();
        MapPointDBEntry(MapPoint* pMP, KeyFrame* pKF_ref_, int idx);  //mimics mappoint constructor
        void addObservation(KeyFrame* pKF, size_t idx);
        bool eraseObservation(KeyFrame* pKF);  //returns whether mappoint is bad - can happen if all observations erased
        void eraseAllObservations(); //used when mappoint is set bad so also clears matches in keyframes
        bool isInKeyFrame(KeyFrame* pKF) ;
        std::map<KeyFrame*,size_t>  getObservations()  { return observations; }
        int getNObs()  {return n_obs;}
        void setNObs(int n);


        void setBestDescriptor( FeatureDescriptor best_desc);
        void addDescriptor(KeyFrame* pKF, FeatureDescriptor desc);
        void eraseDescriptor(KeyFrame* pKF);

        FeatureDescriptor getDescriptor();
        std::map<KeyFrame*, FeatureDescriptor>  getAllDescriptors()  { return descriptors; }
        void computeDistinctiveDescriptor();

        void UpdateNormalAndDepth();
        cv::Mat getNormal();
        void setNormal(cv::Mat norm);
        void setMaxDist(float maxd);
        void setMinDist(float mind);
        float getMaxDist() {return max_distance;}
        float getMinDist() {return min_distance;}

        void setRefKF(KeyFrame* pKF);
        KeyFrame* getRefKF() {return pKF_ref;}

    private:
        MapPoint* pMP_entry;
        KeyFrame* pKF_ref = nullptr;
        std::map<KeyFrame*, size_t>  observations;
        std::map<KeyFrame*, FeatureDescriptor> descriptors;
        cv::Mat normal_vector;
        FeatureDescriptor best_descriptor;
        float min_distance = 0;  //scale invariance distances
        float max_distance = 0;
        int n_obs = 0;
        
        bool desc_needs_update = false;
        bool normdepth_needs_update = false;

        std::mutex entry_mutex;
    };

    class MapPointDB{
    public:
        using MapPointDB_t = std::map< MapPoint*, std::unique_ptr<MapPointDBEntry> >;
        bool inDB(MapPoint* pMP);
        std::vector<MapPoint*> getAllMapPoints();
        long unsigned int numMapPoints();
        int addEntry(MapPoint* pMP, KeyFrame* pKF_ref, int idx);
        int eraseEntry(MapPoint* pMP);  //does the work of SetBadFlag
        int updateEntry(MapPoint* pMP); // update normal and depth
        int addObservation(MapPoint* pMP, KeyFrame* pKF, size_t idx); //also add descriptor
        bool eraseObservation(MapPoint* pMP, KeyFrame* pKF); // also erase descriptor, //returns whether mappoint is bad - can happen if all observations erased
        int replace(MapPoint* pMP_old, MapPoint* pMP_new);
        void clear();

        void validateMapPointDB();


    private:
        MapPointDB_t mappoint_db;
        std::mutex db_mutex;

    };

}//end namespace
#endif
