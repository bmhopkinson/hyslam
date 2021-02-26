#include <CovisibilityGraph.h>

#include <MapPoint.h>
#include <list>

namespace HYSLAM{
    CovisNode::CovisNode(){}//default constructor
    
    CovisNode::CovisNode(KeyFrame* pKF): pKF_node(pKF){  //more typical constructor
    }

    void CovisNode::AddConnection(KeyFrame *pKF, const int &weight)
    {
        //std::cout << "adding connection to : " << pKF->mnId << std::endl;
        {
            std::unique_lock<std::mutex> lock(node_mutex);
            /*  do this check at at higher level - probably in Map or KeyframeDB
            if(pKF->camera.camName != camera.camName){  //only connect keyframes from the same camera
                return;
            }
             */
            if(!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF]=weight;
            else if(mConnectedKeyFrameWeights[pKF]!=weight)
                mConnectedKeyFrameWeights[pKF]=weight;
            else
                return;
        }
//  std::cout << "about to updatebest covisibles for : " << mnId << std::endl;
        UpdateBestCovisibles();
    }

    void CovisNode::EraseConnection(KeyFrame* pKF)
    {
        bool bUpdate = false;  //may want to make updating an option for repeated calls to this func
        {
            std::unique_lock<std::mutex> lock(node_mutex);
            if(mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate=true;
            }
        }

        if(bUpdate)
            UpdateBestCovisibles();
    }

    void CovisNode::UpdateConnections(std::map<KeyFrame*, int> &symmetric_updates ) //"updates" are for other nodes and need to be handled by the CovisGraph
    {
//	std::cout << "in update connections for KF: " << mnId << ", camera: " <<camera.camName << std::endl;
        std::map<KeyFrame*,int> KFcounter;

        std::set<MapPoint*>  spMP;
        spMP = pKF_node->GetMapPoints();

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for(auto vit=spMP.begin(), vend=spMP.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;

            if(pMP->isBad())
                continue;

            std::map<KeyFrame*,size_t> observations = pMP->GetObservations();

            for(std::map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame* pKF_obs = mit->first;
                if(pKF_obs->mnId==pKF_node->mnId)
                    continue;
               if(pKF_obs->isBad()){
				 //  std::cout << "attempt to add bad KF: " << pKF_obs->mnId << ", via mpt: " << pMP->mnId << std::endl;

                    continue;
               }
                KFcounter[mit->first]++;
            }
        }
        //  std::cout << "finished counting keyframe shared mpts: counter size " << KFcounter.size() << std::endl;
        // This should not happen
        if(KFcounter.empty())
            return;

//    std::cout << "counter was not empty, it's size is: " << KFcounter.size() <<  std::endl;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax=0;
        KeyFrame* pKFmax=NULL;
        int th = 15;

        std::vector<std::pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(KFcounter.size());
        //  std::cout << "about to enter loop to add connections" << std::endl;
        for(std::map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
        {
            //	std::cout << "assessing connection for pKFi: " << mit->first->mnId << std::endl;
            if(mit->second>nmax)
            {
                nmax=mit->second;
                pKFmax=mit->first;
            }
            if(mit->second>=th)
            {
                vPairs.push_back(make_pair(mit->second,mit->first));
                symmetric_updates.insert(*mit);
              //  (mit->first)->AddConnection(pKF_node,mit->second);
            }
        }
        //   std::cout << "finished adding connections" << std::endl;
        if(vPairs.empty())
        {
            vPairs.push_back(make_pair(nmax,pKFmax));
            //pKFmax->AddConnection(pKF_node ,nmax);
            symmetric_updates.insert({pKFmax, nmax});
        }

        sort(vPairs.begin(),vPairs.end());
        std::list<KeyFrame*> lKFs;
        std::list<int> lWs;
        for(size_t i=0; i<vPairs.size();i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            std::unique_lock<std::mutex> lockCon(node_mutex);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
            pKF_node->setCovisibleKeyFrames(mvpOrderedConnectedKeyFrames); //push down to keyframes where the data is needed sometimes
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        }
        //  std::cout << "finished updating connections" << std::endl;
    }

    void CovisNode::UpdateBestCovisibles()
    {
        std::unique_lock<std::mutex> lock(node_mutex);
        std::vector<std::pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for(std::map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
            vPairs.push_back(make_pair(mit->second,mit->first));

        sort(vPairs.begin(),vPairs.end());
        std::list<KeyFrame*> lKFs;
        std::list<int> lWs;
        for(size_t i=0, iend=vPairs.size(); i<iend;i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        pKF_node->setCovisibleKeyFrames(mvpOrderedConnectedKeyFrames); //push down to keyframes where the data is needed sometimes
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame*> CovisNode::GetConnectedKeyFrames()
    {
        std::unique_lock<std::mutex> lock(node_mutex);
        std::set<KeyFrame*> s;
        for(std::map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
            s.insert(mit->first);
        return s;
    }

    std::vector<KeyFrame*> CovisNode::GetVectorCovisibleKeyFrames()
    {
        std::unique_lock<std::mutex> lock(node_mutex);
        return mvpOrderedConnectedKeyFrames;
    }

    std::vector<KeyFrame*> CovisNode::GetBestCovisibilityKeyFrames(const int &N)
    {
        std::unique_lock<std::mutex> lock(node_mutex);
        if((int)mvpOrderedConnectedKeyFrames.size()<N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

    }

    std::vector<KeyFrame*> CovisNode::GetCovisiblesByWeight(const int &w)
    {
        std::unique_lock<std::mutex> lock(node_mutex);

        if(mvpOrderedConnectedKeyFrames.empty())
            return std::vector<KeyFrame*>();

        std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,CovisNode::weightComp);  
        if(it==mvOrderedWeights.end())
            return std::vector<KeyFrame*>();
        else
        {
            int n = it-mvOrderedWeights.begin();
            return std::vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
        }
    }

    int CovisNode::GetWeight(KeyFrame *pKF)
    {
        std::unique_lock<std::mutex> lock(node_mutex);
        if(mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    // COVISIBILITY GRAPH FUNCTIONS ///
    bool CovisibilityGraph::inGraph(KeyFrame* pKF){
        CovisibilityGraph_t::const_iterator it;
       // std::map<KeyFrame*, CovisNode>::const_iterator it;
        it = covis_graph.find(pKF);
        if(it != covis_graph.end()){
            return true;
        }
        else {
            return false;
        }
    }

    int CovisibilityGraph::addNode(KeyFrame* pKF){
        if(inGraph(pKF)){
            return -1;
        }
        else{
            {
              std::unique_lock<std::mutex> lock(graph_mutex);
              covis_graph.insert({pKF, std::make_unique<CovisNode>(pKF)});
            }
            
            UpdateConnections(pKF);
            return 0;
        }

    }

    int CovisibilityGraph::eraseNode(KeyFrame* pKF){
        if( !inGraph(pKF) ){
            return -1;
        }
        else{
            {
                std::unique_lock<std::mutex> lock(graph_mutex);
                std::set<KeyFrame*> KFedges = covis_graph[pKF]->GetConnectedKeyFrames();

                //erase all edges in pKF node and all edges connecting from destination nodes to pKF
                for(auto it = KFedges.begin(); it != KFedges.end(); ++it){
                    KeyFrame* pKF_dest = *it;
                    covis_graph[pKF]->EraseConnection(pKF_dest); //probably don't need to do this -

                    if( inGraph(pKF_dest) ){
                        covis_graph[pKF_dest]->EraseConnection(pKF);
                    }
                }

                // now that edges have been cleared, remove node from covis graph
                  covis_graph.erase(pKF);
            }
            return 0;
        }
    }
    
    
    int CovisibilityGraph::UpdateConnections(KeyFrame* pKF){
        if( !inGraph(pKF) ){
            return -1;
        }
        else{
            std::map<KeyFrame*, int> updates;
            covis_graph[pKF]->UpdateConnections(updates);
            for(auto it = updates.begin(); it != updates.end(); ++it){
                KeyFrame* pKFdest = it->first;
                if(inGraph(pKFdest)){  //shouldn't need to do this check
                    covis_graph[pKFdest]->AddConnection(pKF, it->second);
                }
            }
            return 0;
        }
    }
    
    std::set<KeyFrame*> CovisibilityGraph::GetConnectedKeyFrames(KeyFrame* pKF){
        std::set<KeyFrame*> empty;
        if( !inGraph(pKF) ){
            return empty;
        } 
        else{
            std::unique_lock<std::mutex> lock(graph_mutex);
            return covis_graph[pKF]->GetConnectedKeyFrames();
        }
        
    }
    
    std::vector<KeyFrame* > CovisibilityGraph::GetVectorCovisibleKeyFrames(KeyFrame* pKF){
        std::vector<KeyFrame*> empty;
        if( !inGraph(pKF) ){
            return empty;
        } 
        else{
            std::unique_lock<std::mutex> lock(graph_mutex);
            return covis_graph[pKF]->GetVectorCovisibleKeyFrames();
        }
        
    }
    
    std::vector<KeyFrame*> CovisibilityGraph::GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N){
        std::vector<KeyFrame*> empty;
        if( !inGraph(pKF) ){
            return empty;
        } 
        else{
            std::unique_lock<std::mutex> lock(graph_mutex);
            return covis_graph[pKF]->GetBestCovisibilityKeyFrames(N);
        }
    }
    
    std::vector<KeyFrame*> CovisibilityGraph::GetCovisiblesByWeight(KeyFrame* pKF, const int &w){
        std::vector<KeyFrame*> empty;
        if( !inGraph(pKF) ){
            return empty;
        } 
        else{
            std::unique_lock<std::mutex> lock(graph_mutex);
            return covis_graph[pKF]->GetCovisiblesByWeight(w);
        }
    }
    
    int CovisibilityGraph::GetWeight(KeyFrame* pKFnode, KeyFrame* pKFquery){
        if( !inGraph(pKFnode) ){
            return -1;
        } 
        else{
            return covis_graph[pKFnode]->GetWeight(pKFquery);  //returns 0 if not connected
        }
    }
/*
    void CovisibilityGraph::validateCovisiblityGraph(){
        int n_kfs = 0;
        int n_errors = 0;
        int n_correct = 0;
        for(auto it = covis_graph.begin(); it != covis_graph.end(); ++it){
            KeyFrame* pKF = (*it).first;
            if(pKF->isBad())
                continue;
          //  std::cout << "VALIDATE KEYFRAME: " << pKF->mnId << std::endl;
            ++n_kfs;
            int N = 10;
            std::vector<KeyFrame*> KFcovis = pKF->GetBestCovisibilityKeyFrames(N);
            std::vector<KeyFrame*> CGcovis = (*it).second->GetBestCovisibilityKeyFrames(N);

            for(int i = 0; i < N; i++){
                bool KFfound = false;
                bool CGfound = false;
                if(i < static_cast<int>(KFcovis.size())){
                    KFfound = true;
             //       std::cout << "KFcovis: i: "  << i << " , mnId "  << KFcovis[i]->mnId << " " << pKF->GetWeight(KFcovis[i]) << std::endl;
                }
                if(i < static_cast<int>(CGcovis .size())){
                    CGfound = true;
               //     std::cout << "CGcovis: i: "  << i << " , mnId "  << CGcovis[i]->mnId << " " << covis_graph[pKF]->GetWeight(CGcovis[i])  << std::endl;
                }
                if(KFfound && CGfound){
                    if(KFcovis[i]->mnId == CGcovis[i]->mnId ) { //are keyframes the same
                        if (pKF->GetWeight(KFcovis[i]) == covis_graph[pKF]->GetWeight(CGcovis[i])) { //do they have the same weights
                            n_correct++;
                        } else {n_errors++;}
                    } else {n_errors++;}
                }
                else {
                    if(KFfound || CGfound) {  //if one of the datastructures has an edge but the other doesn't - that's an error, if neither have an edge - that's not an error
                        n_errors++;
                    }
                }
            }
        }
     //   std::cout << "FINISHED VALIDATING COVIS GRAPH, N_KFS: " << n_kfs <<  ", n_correct:  "<< n_correct  <<" , n_errors: " << n_errors << std::endl;
    }
*/
} //end namespace
