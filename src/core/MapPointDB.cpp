#include <MapPointDB.h>
#include <FeatureViews.h>
#include <FeatureMatcher.h>

namespace HYSLAM{
    // MAPPOINTDBENTRY MEMBER FUNCTIONS ////
    MapPointDBEntry::MapPointDBEntry(){}
    MapPointDBEntry::MapPointDBEntry(MapPoint* pMP, KeyFrame* pKF_ref_, int idx): pMP_entry(pMP) {
        normal_vector = cv::Mat::zeros(3,1,CV_32F);
        _setRefKF_(pKF_ref_);
        addObservation(pKF_ref_, idx, true);
        FeatureDescriptor descriptor = pKF_ref_->getViews().descriptor(idx);
        addDescriptor(pKF_ref_, descriptor);
        _setBestDescriptor_(descriptor); //only descriptor right now
        _updateMeanDistance_();
        _updateSize_();
        _updateNormalAndDepth_();

    }

    void MapPointDBEntry::addObservation(KeyFrame* pKF, size_t idx, bool replace){
        std::unique_lock<std::mutex> lock(entry_mutex);
        if(observations.count(pKF) && !replace) //don't allow replacement
            return;
        observations[pKF]=idx;
        if(pKF->getViews().uR(idx)>=0)  //seems like this could just be passed in as a parameter (and saved for erasing)
            n_obs+=2;
        else
            n_obs++;

        normdepth_needs_update = true;

        pMP_entry->setObservations(observations);
        pMP_entry->setNObs(n_obs);
       // updateMeanDistance();
        //updateSize();
        _updateEntry_();

    }

    bool MapPointDBEntry::eraseObservation(KeyFrame* pKF){
        std::unique_lock<std::mutex> lock(entry_mutex);
        bool is_bad=false;
        KeyFrame* pRefKF_new = nullptr ;

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


        if(pRefKF_new){
            _setRefKF_(pRefKF_new);
        }

        _updateEntry_();

        return is_bad;
    }

    void MapPointDBEntry::eraseAllObservations(){
        std::unique_lock<std::mutex> lock(entry_mutex);
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
        std::unique_lock<std::mutex> lock(entry_mutex);
        return observations.count(pKF);
    }

    void MapPointDBEntry::_setBestDescriptor_(FeatureDescriptor bd){
        best_descriptor = bd;
        pMP_entry->setDescriptor(bd);
    }

    FeatureDescriptor MapPointDBEntry::getDescriptor(){
        std::unique_lock<std::mutex> lock(entry_mutex);
        if(desc_needs_update){
            _computeDistinctiveDescriptor_();
            desc_needs_update = false;
        }

        return best_descriptor;
        
    }


    void MapPointDBEntry::addDescriptor(KeyFrame* pKF, FeatureDescriptor d){
        std::unique_lock<std::mutex> lock(entry_mutex);
        if(descriptors.count(pKF)) //don't allow replacement
            return;
        descriptors[pKF] = d;
        desc_needs_update = true;
    }

    void MapPointDBEntry::eraseDescriptor(KeyFrame* pKF){
        std::unique_lock<std::mutex> lock(entry_mutex);
        if(descriptors.count(pKF)){
            descriptors.erase(pKF);
            desc_needs_update = true;

        }
    }

    void MapPointDBEntry::_computeDistinctiveDescriptor_(){
        // Compute distances between them -
        std::vector<FeatureDescriptor> descriptor_vec;
        {
            for(auto it = descriptors.begin(); it != descriptors.end(); ++it){
                if(!it->first->isBad()){
                    descriptor_vec.push_back(it->second);
                }
            }
        }  //copied relevant data

        if(descriptor_vec.empty()){
            return;
        }

        const size_t N = descriptor_vec.size();

        float Distances[N][N];
        for(size_t i=0;i<N;i++)
        {
            FeatureDescriptor desc_i = descriptor_vec[i];
            Distances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                float distij =desc_i.distance(descriptor_vec[j]);
                Distances[i][j]=distij;
                Distances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        float BestMedian = std::numeric_limits<float>::max();
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            std::vector<int> vDists(Distances[i],Distances[i]+N);
            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        _setBestDescriptor_( descriptor_vec[BestIdx] );
    }


    void MapPointDBEntry::_setNormal_(cv::Mat norm){
        normal_vector = norm.clone();
        pMP_entry->setNormal(norm);

    }
    
    cv::Mat MapPointDBEntry::getNormal(){
        std::unique_lock<std::mutex> lock(entry_mutex);
        if(normdepth_needs_update){
            _updateNormalAndDepth_();
        }
        return normal_vector.clone();
    }

    void MapPointDBEntry::_setMaxDist_(float maxd){
        max_distance = maxd;
        pMP_entry->setMaxDistanceInvariance(maxd);

    }
    void MapPointDBEntry::_setMinDist_(float mind){
        min_distance = mind;
        pMP_entry->setMinDistanceInvariance(mind);
    }

    void MapPointDBEntry::_setRefKF_(KeyFrame* pKF){
        pKF_ref = pKF;
        pMP_entry->setReferenceKeyFrame(pKF);
    }

    void MapPointDBEntry::_setMeanDistance_(float dist){
        mean_distance = dist;
        pMP_entry->setMeanDistance(mean_distance);
    }

    void MapPointDBEntry::_setSize_(float size_){
        size = size_;
        pMP_entry->setSize(size);
    }

    void MapPointDBEntry::updateEntry() {
        std::unique_lock<std::mutex> lock(entry_mutex);
        _updateEntry_();

    }

    void MapPointDBEntry::_updateEntry_(){
        _updateNormalAndDepth_();
        _computeDistinctiveDescriptor_();
        _updateMeanDistance_();
        _updateSize_();
    }

    void MapPointDBEntry::_updateNormalAndDepth_(){
        std::map<KeyFrame*,size_t> observations_cur;
        KeyFrame* pRefKF;
        cv::Mat Pos;

        {
            observations_cur=observations;
            pRefKF=pKF_ref;
            Pos = pMP_entry->GetWorldPos();
        }//copied relevant data

        if(observations_cur.empty())
            return;

        cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
        int n=0;
        for(std::map<KeyFrame*,size_t>::iterator mit=observations_cur.begin(), mend=observations_cur.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = Pos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const float dist = cv::norm(PC);

        float max_distance_ = max_dist_invariance_factor * dist;
        float min_distance_ = min_dist_invariance_factor * dist;

        _setMaxDist_(max_distance_);
        _setMinDist_(min_distance_);
        _setNormal_(normal/n);
    }

    void MapPointDBEntry::_updateMeanDistance_(){

        cv::Mat Pos;
        Pos = pMP_entry->GetWorldPos();


        if(observations.empty())
            return;

        //cv::Mat mean_dist = cv::Mat::zeros(3,1,CV_32F);
        float mean_dist = 0.0;
        int n=0;
        for(std::map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat Pos_cam = Pos - Owi;
            float this_dist = cv::norm(Pos_cam);
            mean_dist  += this_dist;
      //      std::cout << "lm dist: " << this_dist << "\t";
            n++;
        }
        mean_dist = mean_dist/static_cast<float>(n);
        _setMeanDistance_(mean_dist);
     //   std::cout << ", mean dist: " << mean_dist << std::endl;
    }

    void MapPointDBEntry::_updateSize_(){

        int n=0;
        float mean_size = 0.0;
        for(std::map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++) {
            KeyFrame *pKF = mit->first;
            size_t idx = mit->second;
            float size_this = pKF->featureSizeMetric(idx);
        //    std::cout << "size update got size" <<std::endl;
            if (size_this > 0.0) {
                mean_size += size_this;
                n++;
           //     std::cout << "size: " << size_this << "\t";
            }

        }

        mean_size = mean_size/static_cast<float>(n);
        _setSize_(mean_size);
       // std::cout << ", mean size: " << mean_size << std::endl;
    }


    // MAPPOINTDB MEMBER FUNCTIONS ////

    bool MapPointDB::inDB(MapPoint* pMP){
        if(mappoint_db.count(pMP)){
            return true;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                if((*it)->inDB(pMP)){
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<MapPoint*> MapPointDB::getAllMapPoints() {
        std::vector<MapPoint*> all_mps;
        all_mps.reserve(mappoint_db.size());
        for(auto it = mappoint_db.begin(); it != mappoint_db.end(); ++it){
            all_mps.push_back(it->first);
        }

        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
            std::vector<MapPoint*> all_sub = (*it)->getAllMapPoints();
            all_mps.reserve(all_mps.size() + std::distance(all_sub.begin(), all_sub.end()));
            all_mps.insert(all_mps.end(),all_sub.begin(), all_sub.end() );
        }
        return all_mps;
    }

    long unsigned int MapPointDB::numMapPoints(){
        long unsigned int total = mappoint_db.size();
        for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it) {
          total=+ (*it)->numMapPoints();
        }
        return total;
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
        if(mappoint_db.count(pMP)){
            std::unique_lock <std::mutex> lock(db_mutex);
            _eraseEntry_(pMP);
            return 0;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                if((*it)->eraseEntry(pMP) == 0){
                    return 0;
                }
            }
        }
        return -1;
    }


    int MapPointDB::_eraseEntry_(MapPoint *pMP) {
        pMP->setBad();
        mappoint_db[pMP]->eraseAllObservations();
        mappoint_db.erase(pMP);
        return 0;
    }

    int MapPointDB::updateEntry(MapPoint* pMP) {
        if(mappoint_db.count(pMP)){
            std::unique_lock <std::mutex> lock(db_mutex);
            mappoint_db[pMP]->updateEntry();
            return 0;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                if((*it)->updateEntry(pMP) == 0){
                    return 0;
                }
            }
        }
        return -1;
    }

    int MapPointDB::addObservation(MapPoint* pMP, KeyFrame* pKF, size_t idx, bool replace){

        if(mappoint_db.count(pMP)){
            mappoint_db[pMP]->addObservation(pKF, idx, replace);
            FeatureDescriptor desc = pKF->getViews().descriptor(idx);
            mappoint_db[pMP]->addDescriptor(pKF, desc);
            return 0;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                if((*it)->addObservation( pMP, pKF, idx, replace) == 0){
                    return 0;
                }
            }
        }
        return -1;

    }

    int MapPointDB::eraseObservation(MapPoint* pMP, KeyFrame* pKF){ //returns whether mappoint is bad - can happen if all observations erased

        if(mappoint_db.count(pMP)){
            std::unique_lock <std::mutex> lock(db_mutex);
            bool to_erase = mappoint_db[pMP]->eraseObservation(pKF);
            mappoint_db[pMP]->eraseDescriptor(pKF);
            if (to_erase && !pMP->Protected()) {
                _eraseEntry_(pMP);
                return 1;
            }
            return 0;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                int res = (*it)->eraseObservation(pMP, pKF);
                if( res >= 0){
                    return res;
                }
            }
        }
        return -1;
    }

    std::map<KeyFrame *, size_t> MapPointDB::getObservations(MapPoint *pMP) {
        std::map<KeyFrame *, size_t> obs;
        if(mappoint_db.count(pMP)){
            std::unique_lock<std::mutex> lock1(db_mutex);
            obs = mappoint_db[pMP]->getObservations();
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                obs = (*it)->getObservations(pMP);
                if( !obs.empty()){
                    return obs;
                }
            }
        }
        return obs;
    }

    int MapPointDB::replace(MapPoint* pMP_old, MapPoint* pMP_new) {
        if (pMP_old->mnId == pMP_new->mnId)
            return -1;

        if (!inDB(pMP_old) || !inDB(pMP_new))
            return -1;

        {
            std::unique_lock<std::mutex> lock1(db_mutex);  //lock to set bad so no new observations are added
            pMP_old->setBad();
            pMP_old->setReplaced(pMP_new);
        }

        std::map<KeyFrame *, size_t> obs= getObservations(pMP_old);
        MapPointDBEntry* entry_pMPnew = _findEntry_(pMP_new);

        for (std::map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            if (!entry_pMPnew->isInKeyFrame(pKF)) {
                pKF->associateLandMark(mit->second, pMP_new, true);
                addObservation(pMP_new, pKF, mit->second, true);
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
        for(auto it= sub_dbs.begin(); it != sub_dbs.end(); ++it){
            (*it)->clear();
        }
    }

    bool MapPointDB::exists(MapPoint* pMP) {
        if(mappoint_db.count(pMP)){
            return true;
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                if((*it)->inDB(pMP)){
                    return true;
                }
            }
        }
        return false;
    }

    MapPointDBEntry *MapPointDB::_findEntry_(MapPoint *pMP) {
        MapPointDBEntry* entry = nullptr;
        if(mappoint_db.count(pMP)){
            return mappoint_db[pMP].get();
        }
        else{
            for(auto it = sub_dbs.begin(); it != sub_dbs.end(); ++it){
                entry = (*it)->_findEntry_(pMP);
                if(entry){
                    return entry;
                }
            }
        }
        return nullptr;
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

    void MapPointDB::addChild(std::shared_ptr<MapPointDB> child) {
        sub_dbs.push_back(child);
    }

    void MapPointDB::removeChild(std::shared_ptr<MapPointDB> child) {
        sub_dbs.remove(child);
    }


} //end namespace

