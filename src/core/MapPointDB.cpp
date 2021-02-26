#include <MapPointDB.h>
#include <ORBViews.h>
#include <ORBmatcher.h>

namespace HYSLAM{
    // MAPPOINTDBENTRY MEMBER FUNCTIONS ////
    MapPointDBEntry::MapPointDBEntry(){}
    MapPointDBEntry::MapPointDBEntry(MapPoint* pMP, KeyFrame* pKF_ref_, int idx): pMP_entry(pMP) {
        normal_vector = cv::Mat::zeros(3,1,CV_32F);
        setRefKF(pKF_ref_);
        addObservation(pKF_ref_, idx);
        cv::Mat descriptor = pKF_ref_->getViews().descriptor(idx);
        addDescriptor(pKF_ref_, descriptor);
        setBestDescriptor(descriptor); //only descriptor right now
        UpdateNormalAndDepth();

    }

    void MapPointDBEntry::addObservation(KeyFrame* pKF, size_t idx){
        std::unique_lock<mutex> lock(entry_mutex);
        if(observations.count(pKF)) //don't allow replacement
            return;
        observations[pKF]=idx;
        if(pKF->getViews().uR(idx)>=0)  //seems like this could just be passed in as a parameter (and saved for erasing)
            n_obs+=2;
        else
            n_obs++;

        normdepth_needs_update = true;

        pMP_entry->setObservations(observations);
        pMP_entry->setNObs(n_obs);

    }

    bool MapPointDBEntry::eraseObservation(KeyFrame* pKF){
        bool is_bad=false;
        KeyFrame* pRefKF_new = nullptr ;
        {
            std::unique_lock<mutex> lock(entry_mutex);
            if(observations.count(pKF))
            {
                int idx = observations[pKF];
                if(pKF->getViews().uR(idx)>=0)
                    n_obs-=2;
                else
                    n_obs--;

                observations.erase(pKF);
                normdepth_needs_update = true;


                if(pKF_ref==pKF)
                    pRefKF_new=observations.begin()->first;

                // If only 2 observations or less, discard point
                if((n_obs <= 2) && !pMP_entry->Protected()) {
                    is_bad = true;
                }
            }
            pMP_entry->setObservations(observations);
            pMP_entry->setNObs(n_obs);

        }
        if(pRefKF_new){
            setRefKF(pRefKF_new);  //this acquires lock on entry_mutex so can't do in above block
        }

        return is_bad;
    }

    void MapPointDBEntry::eraseAllObservations(){
        std::unique_lock<mutex> lock(entry_mutex);
        for(auto it = observations.begin(); it != observations.end(); ++it){
             KeyFrame* pKF = it->first;
             size_t idx = it->second;
             pKF->removeLandMarkAssociation(idx);
        }
        observations.clear();
        descriptors.clear();
        pMP_entry->setObservations(observations);
    }

    bool MapPointDBEntry::isInKeyFrame(KeyFrame* pKF){
        std::unique_lock<mutex> lock(entry_mutex);
        return observations.count(pKF);
    }

    void MapPointDBEntry::setBestDescriptor(cv::Mat bd){
        std::unique_lock<mutex> lock(entry_mutex);
        best_descriptor = bd.clone();
        pMP_entry->setDescriptor(bd);
    }
    
    cv::Mat MapPointDBEntry::getDescriptor(){
        if(desc_needs_update){
            computeDistinctiveDescriptor();
            desc_needs_update = false;
        }
        std::unique_lock<mutex> lock(entry_mutex);
        return best_descriptor.clone();
        
    }


    void MapPointDBEntry::addDescriptor(KeyFrame* pKF, cv::Mat d){
        std::unique_lock<mutex> lock(entry_mutex);
        if(descriptors.count(pKF)) //don't allow replacement
            return;
        descriptors[pKF] = d.clone();
        desc_needs_update = true;
    }

    void MapPointDBEntry::eraseDescriptor(KeyFrame* pKF){
        std::unique_lock<mutex> lock(entry_mutex);
        if(descriptors.count(pKF)){
            descriptors.erase(pKF);
            desc_needs_update = true;

        }
    }

    void MapPointDBEntry::computeDistinctiveDescriptor(){
        // Compute distances between them -
        vector<cv::Mat> descriptor_vec;
        {
            std::unique_lock<mutex> lock(entry_mutex);
            for(auto it = descriptors.begin(); it != descriptors.end(); ++it){
                if(!it->first->isBad()){
                    descriptor_vec.push_back(it->second);
                }
            }
        }  //copied relevant data - release lock

        if(descriptor_vec.empty()){
            return;
        }

        const size_t N = descriptor_vec.size();

        float Distances[N][N];
        for(size_t i=0;i<N;i++)
        {
            Distances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                int distij = ORBmatcher::DescriptorDistance(descriptor_vec[i],descriptor_vec[j]);
                Distances[i][j]=distij;
                Distances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists(Distances[i],Distances[i]+N);
            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        setBestDescriptor( descriptor_vec[BestIdx] );
    }


    void MapPointDBEntry::setNObs(int n){
        pMP_entry->setNObs(n);
    }

    void MapPointDBEntry::setNormal(cv::Mat norm){
        unique_lock<mutex> lock(entry_mutex);
        normal_vector = norm.clone();
        pMP_entry->setNormal(norm);

    }
    
    cv::Mat MapPointDBEntry::getNormal(){
        if(normdepth_needs_update){
            UpdateNormalAndDepth();
        }
        unique_lock<mutex> lock(entry_mutex);
        return normal_vector.clone();
    }

    void MapPointDBEntry::setMaxDist(float maxd){
        unique_lock<mutex> lock(entry_mutex);
        max_distance = maxd;
        pMP_entry->setMaxDistanceInvariance(maxd);

    }
    void MapPointDBEntry::setMinDist(float mind){
        unique_lock<mutex> lock(entry_mutex);
        min_distance = mind;
        pMP_entry->setMinDistanceInvariance(mind);
    }

    void MapPointDBEntry::setRefKF(KeyFrame* pKF){
        unique_lock<mutex> lock(entry_mutex);
        pKF_ref = pKF;
        pMP_entry->setReferenceKeyFrame(pKF);
    }

    void MapPointDBEntry::UpdateNormalAndDepth(){
        map<KeyFrame*,size_t> observations_cur;
        KeyFrame* pRefKF;
        cv::Mat Pos;

        {
            unique_lock<mutex> lock1(entry_mutex);
            observations_cur=observations;
            pRefKF=pKF_ref;
            Pos = pMP_entry->GetWorldPos();
        }//copied relevant data - release lock

        if(observations_cur.empty())
            return;

        cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
        int n=0;
        for(map<KeyFrame*,size_t>::iterator mit=observations_cur.begin(), mend=observations_cur.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = Pos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const float dist = cv::norm(PC);

        const int level = pRefKF->getViews().keypt(observations[pRefKF]).octave;
        const float levelScaleFactor =  pRefKF->getViews().orbParams().mvScaleFactors[level];
        const int nLevels = pRefKF->getViews().orbParams().mnScaleLevels;


        float max_distance_ = dist*levelScaleFactor;
        float min_distance_ = max_distance_/pRefKF->getViews().orbParams().mvScaleFactors[nLevels-1];
        setMaxDist(max_distance_);
        setMinDist(min_distance_);
        setNormal(normal/n);
    }

    // MAPPOINTDB MEMBER FUNCTIONS ////

    bool MapPointDB::inDB(MapPoint* pMP){
        return  mappoint_db.count(pMP);
    }

    std::vector<MapPoint*> MapPointDB::getAllMapPoints() {
        std::vector<MapPoint*> all_mps;
        all_mps.reserve(mappoint_db.size());
        for(auto it = mappoint_db.begin(); it != mappoint_db.end(); ++it){
            all_mps.push_back(it->first);
        }
        return all_mps;
    }

    long unsigned int MapPointDB::numMapPoints(){
        return mappoint_db.size();
    }

    int MapPointDB::addEntry(MapPoint* pMP, KeyFrame* pKF_ref, int idx){
        if(inDB(pMP)){
            return -1;
        }
        else{
            {
             //   std::cout << "adding to MapPointDB, pMP: "  << pMP->mnId << std::endl;
                std::unique_lock<std::mutex> lock(db_mutex);
                mappoint_db.insert({pMP, std::make_unique<MapPointDBEntry>(pMP, pKF_ref, idx)});
            }
            return 0;
        }
    }


    int MapPointDB::eraseEntry(MapPoint* pMP) {   
        if(!inDB(pMP)){
            return -1;
        }
        else {
            {
                std::unique_lock <std::mutex> lock(db_mutex);
                pMP->setBad();
                mappoint_db[pMP]->eraseAllObservations();
                mappoint_db.erase(pMP);
            }
        }
        return 0;
    }

    int MapPointDB::updateEntry(MapPoint* pMP) {
        if(!inDB(pMP)){
            return -1;
        }
        else {
            mappoint_db[pMP]->computeDistinctiveDescriptor();
            mappoint_db[pMP]->UpdateNormalAndDepth();
            return 0;
        }
    }

    int MapPointDB::addObservation(MapPoint* pMP, KeyFrame* pKF, size_t idx){
        if(!inDB(pMP)){
            return -1;
        }
        else {
            mappoint_db[pMP]->addObservation(pKF, idx);
            cv::Mat desc = pKF->getViews().descriptor(idx);
            mappoint_db[pMP]->addDescriptor(pKF, desc);
        }
        return 0;
    }

    bool MapPointDB::eraseObservation(MapPoint* pMP, KeyFrame* pKF){ //returns whether mappoint is bad - can happen if all observations erased
        if(!inDB(pMP)){
            return false;
        }
        else {
            bool to_erase = mappoint_db[pMP]->eraseObservation(pKF);
            mappoint_db[pMP]->eraseDescriptor(pKF);
            if (to_erase && !pMP->Protected()) {
                eraseEntry(pMP);
                return true;
            }
            return false;
        }
    }

    int MapPointDB::replace(MapPoint* pMP_old, MapPoint* pMP_new) {
        if (pMP_old->mnId == pMP_new->mnId)
            return -1;

        if (!inDB(pMP_old) || !inDB(pMP_new))
            return -1;

        std::map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(db_mutex);
            obs = mappoint_db[pMP_old]->getObservations();
            pMP_old->setBad();
            pMP_old->setReplaced(pMP_new);

        }

        for (std::map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            if (!mappoint_db[pMP_new]->isInKeyFrame(pKF)) {
                pKF->associateLandMark(mit->second, pMP_new, true);
                addObservation(pMP_new, pKF, mit->second);
            } else {
                pKF->removeLandMarkAssociation(mit->second);
            }
        }

        updateEntry(pMP_new);
        eraseEntry(pMP_old);

        return 0;
    }

    void MapPointDB::clear(){
        mappoint_db.clear();
    }


    void MapPointDB::validateMapPointDB(){
        std::cout << "VALIDATING MAPPOINTDB"  <<std::endl;

        int n_total = 0;
        int n_diff = 0;
        for(auto it = mappoint_db.begin(); it != mappoint_db.end(); ++it, ++n_total){
            MapPoint* pMP = it->first;
            auto it2 =it;
            it2++;
            for(; it2 != mappoint_db.end(); ++it2){
                MapPoint* pMP2 = it2->first;
                if(pMP2){
                    if(pMP->mnId == pMP2->mnId){
                        std::cout << " seem to have two mappoints: " << pMP->mnId << "\t" << pMP << "\t" << pMP2 << std::endl;
                    }
                }
            }
            //std::cout << "pMP: DB nObs: MP nObs: " << pMP->mnId << "\t" << it->second->getNObs() <<"\t" << pMP->Observations() << std::endl;
          //  if(it->second->getNObs() != pMP->Observations()){
           //     n_diff++;
           // }
            /*
            std::cout << "DB normal: " << pMP->normal_vector_pushed <<std::endl;
            std::cout << "MP normal: " << pMP->GetNormal() <<std::endl;


            std::cout << "DB refKF: " << pMP->mpRefKF_pushed << std::endl;
            std::cout << "MP refKF: " << pMP->GetReferenceKeyFrame() << std::endl;

            std::cout << "DB min_dist: " << pMP->mfMinDistance_pushed << std::endl;
            std::cout << "MP min_dist: " << pMP->GetMinDistanceInvariance()/0.8 << std::endl;
            std::cout << "DB max_dist: " << pMP->mfMaxDistance_pushed<< std::endl;
            std::cout << "MP max_dist: " << pMP->GetMaxDistanceInvariance()/1.2 << std::endl;


            std::map<KeyFrame*,size_t> obs = pMP->observations_pushed;

            if(pMP->isBad()){
                std::cout << "MapPointDB obs of bad mpt: " <<  it->second->getNObs() << std::endl;
            }
            std::cout << "pushed MapPointDB observations: " <<std::endl;
            for(auto it2 = obs.begin(); it2 != obs.end(); ++it2){
                std::cout << "pKF: " << it2->first->mnId << " bad: "<< it2->first->isBad() <<", idx: " <<it2->second << std::endl;
            }

            std::cout << "Direct MapPoint observations: " <<std::endl;
            std::map<KeyFrame*,size_t> obs_mp =  pMP->GetObservations();
            for(auto it3 = obs_mp.begin(); it3 != obs_mp.end(); ++it3){
                std::cout << "pKF: " << it3->first->mnId << " bad: "<< it3->first->isBad() << ", idx: " <<it3->second << std::endl;
            }

            */


            /*
            std::cout << "DB refKF: " << it->second->getRefKF() << std::endl;
            std::cout << "MP refKF: " << pMP->GetReferenceKeyFrame() << std::endl;

            std::cout << "DB min_dist: " << it->second->getMinDist() << std::endl;
            std::cout << "MP min_dist: " << pMP->GetMinDistanceInvariance()/0.8 << std::endl;
            std::cout << "DB max_dist: " << it->second->getMaxDist()<< std::endl;
            std::cout << "MP max_dist: " << pMP->GetMaxDistanceInvariance()/1.2 << std::endl;
       //     std::cout << "validating mappoint: " << pMP->mnId << ", bad?" << pMP->isBad() << std::endl;
       */
        /*
            cv::Mat des_db = it->second->getDescriptor() ;
            cv::Mat des_mp =  pMP->GetDescriptor() ;
            cv::Mat diff = des_db != des_mp;
            
            if(cv::countNonZero(diff) !=0){
                ++n_diff;
              std::cout << "DB desc: " << it->second->getDescriptor() <<std::endl;
              std::cout << "MP desc: " << pMP->GetDescriptor() <<std::endl;
              std::cout << "DB normal: " << it->second->getNormal() <<std::endl;
              std::cout << "MP normal: " << pMP->GetNormal() <<std::endl;
              
               std::map<KeyFrame*, cv::Mat> desc = it->second->getAllDescriptors();

              if(pMP->isBad()){
                std::cout << "MapPointDB desc of bad mpt: " <<  it->second->getNObs() << std::endl;

              }
              std::cout << "MapPointDB descriptors: " <<std::endl;
              for(auto it2 = desc.begin(); it2 != desc.end(); ++it2){
                std::cout << "pKF: " << it2->first->mnId << " bad: "<< it2->first->isBad() <<", idx: " <<it2->second << std::endl;
              }

              std::cout << "Direct MapPoint descriptors: " <<std::endl;
              std::map<KeyFrame*, cv::Mat> desc_mp =  pMP->GetAllDescriptors();
              for(auto it3 = desc_mp.begin(); it3 != desc_mp.end(); ++it3){
                std::cout << "pKF: " << it3->first->mnId << " bad: "<< it3->first->isBad() << ", idx: " <<it3->second << std::endl;
              }
              
              
            }
        */
/*

            
        */    
        /*
            std::map<KeyFrame*,size_t> obs = it->second->getObservations();

            if(pMP->isBad()){
                std::cout << "MapPointDB obs of bad mpt: " <<  it->second->getNObs() << std::endl;

            }
             std::cout << "MapPointDB observations: " <<std::endl;
            for(auto it2 = obs.begin(); it2 != obs.end(); ++it2){
                std::cout << "pKF: " << it2->first->mnId << " bad: "<< it2->first->isBad() <<", idx: " <<it2->second << std::endl;
            }

            std::cout << "Direct MapPoint observations: " <<std::endl;
            std::map<KeyFrame*,size_t> obs_mp =  pMP->GetObservations();
            for(auto it3 = obs_mp.begin(); it3 != obs_mp.end(); ++it3){
                std::cout << "pKF: " << it3->first->mnId << " bad: "<< it3->first->isBad() << ", idx: " <<it3->second << std::endl;
            }
            */
        }
        std::cout <<"finished w/ MapPointDB, total mpts: " <<n_total << " , diff: " << n_diff <<std::endl;
        
    }

} //end namespace

