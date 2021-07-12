#ifndef HYSLAM_MAPPOINTDB_H_
#define HYSLAM_MAPPOINTDB_H_

/*
 *
 * class that holds information about LandMarks (mappoints) and their associations with KeyFrame features
 * pushes down data (observations, normal, size, etc) to individual MapPoints so that data is available when needed
 * MapPointDB contains MapPointDBEntry for each LandMark
 * NOTE: might pull out a separate "AssociationsDB" - currently the "MapPointDB" does double duty as a true mappoint database and as an associations database
 *
 * MappointDB is a recursive tree structure to allow sharing (or not) of information between submaps (see Map). function calls to the main db are implemented recursively to search any child MapPointDBs.
 *
 * MapPointDBEntry:
 *    core data are:
 *      Pointer to LandMark itself
 *      Pointer to refernce keyframe (this is only used in LoopClosing and is suspect b/c i don't think it's ever updated if a KeyFrame is set bad)
 *      Observations - Keyframes and associated Feature index where landmark has been observed
 *      Descriptors - Feature descriptors from all KeyFrames in which LandMark has been observed
 *      Best Feature Descriptor - representative feature descriptor for the LandMark
 *      size - radius of the LandMark (in world dimensions e.g. meters) - from mean of all observations
 *      normal - avg viewing direction between LandMark and KeyFrame centers
 *      max and min distance invariance  - range within which the landmark is considered valid for observing in an image
 *
 *    Key Functionality:
 *      public: these public functions lock the entry's mutex and so should not be call internally (though right now some are called in the constructor, which doesn't lock the mutex)
 *      MapPointDBEntry(MapPoint* pMP, KeyFrame* pKF_ref_, int idx);  //mimics mappoint constructor - requires the landmark itself, the first keyframe it's viewed in (which become the reference keyframe) and the feature index corresponding to the LandMark
 *      addObservation(KeyFrame* pKF, size_t idx); - appends new observation of LandMark in pKF at feature idx to existing observations and updates entry (_UpdateEntry_() ), which pushes new data down to LandMark
 *      addDescriptor(KeyFrame* pKF, FeatureDescriptor desc); appends new desciptor - needs to be called with addObservation - these should be combined into a single function (addObservation and addDescriptor)
 *      bool eraseObservation(KeyFrame* pKF);  // removes LandMark observation in pKF (if such an observation exists) returns whether mappoint is bad - can happen if all observations erased,
 *              and updates entry (_UpdateEntry_() ), which pushes new data down to LandMark
 *
 *      private: these functions don't lock the entry's mutex and can only be called internally
 *          _updateEntry_() - updates "everything" by calling         _updateNormalAndDepth_(), _computeDistinctiveDescriptor_(), _updateMeanDistance_(), _updateSize_();
 *          _computeDistinctiveDescriptor_() determines the "Best" (most representative) feature descriptor for the LandMark as the descriptor with the smallest median distance to all other descriptors
 *          _updateNormalAndDepth_() determines the normal as the average viewing direction between LandMark and associated KeyFrame centers, sets depth invariance based on average viewing distance between KeyFrame center and LandMark
 *          _updateSize_() - determines average size of the LandMark through cooperation with KeyFrames in which landmark is viewed
 *          _updateMeanDistance_(); determines mean distance between LandMark and associated KeyFrame centers - much of the code is duplicated in _updateNormalAndDepth_() and this functionality should probably be merged into that function
 *
 * MapPointDB - holds MapPointDB entries (which do the bulk of the work).
 *  most function calls are simply passed on to the respective MapPointDBEntry.
 *  the primary exception is:
 *      int replace(MapPoint* pMP_old, MapPoint* pMP_new) - sets pMP_old as bad and erases from MapPointDB, but first transfers all observations (and associated data) from pMP_old to pMP_new. pMP_new is updated
 *
 *
 */

#include <MapPoint.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <FeatureDescriptor.h>
#include <opencv2/core/core.hpp>

#include <map>
#include <mutex>
#include <list>

namespace HYSLAM{
    class MapPointDBEntry{
    public:
        MapPointDBEntry();
        MapPointDBEntry(MapPoint* pMP, KeyFrame* pKF_ref_, int idx);  //mimics mappoint constructor
        void addObservation(KeyFrame* pKF, size_t idx, bool replace);
        bool eraseObservation(KeyFrame* pKF);  //returns whether mappoint is bad - can happen if all observations erased
        void eraseAllObservations(); //used when mappoint is set bad so also clears matches in keyframes
        bool isInKeyFrame(KeyFrame* pKF) ;
        void updateEntry();
        std::map<KeyFrame*,size_t>  getObservations()  { return observations; }
        int getNObs()  {return n_obs;}

        void addDescriptor(KeyFrame* pKF, FeatureDescriptor desc);
        void eraseDescriptor(KeyFrame* pKF);

        FeatureDescriptor getDescriptor();
        std::map<KeyFrame*, FeatureDescriptor>  getAllDescriptors()  { return descriptors; }

        cv::Mat getNormal();

        float getMaxDist() {return max_distance;}
        float getMinDist() {return min_distance;}

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
        float mean_distance = 0;
        float size = 0; //average size of feature at mean_distance
        int n_obs = 0;
        
        bool desc_needs_update = false;
        bool normdepth_needs_update = false;

        float max_dist_invariance_factor = 2.0;
        float min_dist_invariance_factor = 0.5;

        std::mutex entry_mutex;

        //private functions - don't lock
        void _updateEntry_();
        void _setBestDescriptor_( FeatureDescriptor best_desc);
        void _computeDistinctiveDescriptor_();

        void _setMeanDistance_(float dist);
        void _updateMeanDistance_();

        void _setSize_(float size_);
        void _updateSize_();

        void _setNormal_(cv::Mat norm);
        void _updateNormalAndDepth_();

        void _setRefKF_(KeyFrame* pKF);
        void _setMaxDist_(float maxd);
        void _setMinDist_(float mind);


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
        int addObservation(MapPoint* pMP, KeyFrame* pKF, size_t idx, bool replace); //also add descriptor
        int eraseObservation(MapPoint* pMP, KeyFrame* pKF); // also erase descriptor, //-1 = failed, 0 = erased observation, 1 = erased observation which made mappoint bad
        std::map<KeyFrame *, size_t> getObservations(MapPoint* pMP);
        int replace(MapPoint* pMP_old, MapPoint* pMP_new);
        bool exists(MapPoint* pMP);  //DUPLICATE OF inDB - UNIFY
        void clear();

        void addChild(std::shared_ptr<MapPointDB> child);
        void removeChild(std::shared_ptr<MapPointDB> child);

        void validateMapPointDB();


    private:
        MapPointDB_t mappoint_db;
        std::mutex db_mutex;
        std::list<std::shared_ptr<MapPointDB>> sub_dbs;

        int _eraseEntry_(MapPoint* pMP); //nonlocking
        MapPointDBEntry* _findEntry_(MapPoint* pMP); //use carefully - returns raw pointer to unique pointer

    };

}//end namespace
#endif
