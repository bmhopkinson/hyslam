#ifndef COVISIBILITYGRAPH_H_
#define COVISIBILITYGRAPH_H_

#include <KeyFrame.h>
#include <map>
#include <vector>
#include <set>
#include <mutex>
#include <memory>

namespace HYSLAM{
  class CovisNode{ //right now preserving ORBSLAM2 structure/behavior - node centric graph
    public:
        // Covisibility graph functions
        CovisNode();
        CovisNode(KeyFrame* pKF);
        void AddConnection(KeyFrame* pKF, const int &weight); //only used within KeyFrame (in UpdateBestCovisibles() )
        void EraseConnection(KeyFrame* pKF); // only used within  KeyFrame in SetBadFlag
        void UpdateConnections(std::map<KeyFrame*, int> &symmetric_updates ); //called in Tracking, LocalMapping, Loop Closing, ImagingBA
        void UpdateBestCovisibles(); //only used internally (in KeyFrame) in UpdateBestCovisibles and AddConnection
        void ClearConnections();
        std::set<KeyFrame*> GetConnectedKeyFrames(); //used in LoopClosing and KeyFrameDatabase
        std::vector<KeyFrame* > GetVectorCovisibleKeyFrames(); //used in LocalMapping, LocalBA, and LoopClosing
        std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N); //used in Tracking, KeyFrameDatabase, LocalMapping
        std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w); // used in Optimizer, MapDrawer
        int GetWeight(KeyFrame* pKF); //using in Optimizer and KeyFrame implementation

    private:
        KeyFrame* pKF_node;
        std::map<KeyFrame*,int> mConnectedKeyFrameWeights;  //edges
        std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
        std::vector<int> mvOrderedWeights;
        std::mutex node_mutex;
        
        static bool weightComp( int a, int b){
            return a>b;
        }
  };

  class CovisibilityGraph{
        
    public:
        using CovisibilityGraph_t = std::map< KeyFrame*, std::unique_ptr<CovisNode> >;
        int addNode(KeyFrame* pKF);
        int eraseNode(KeyFrame* pKF);
        int UpdateConnections(KeyFrame* pKF);
        std::set<KeyFrame *> GetConnectedKeyFrames(KeyFrame* pKF); //used in LoopClosing and KeyFrameDatabase
        std::vector<KeyFrame* > GetVectorCovisibleKeyFrames(KeyFrame* pKF); //used in LocalMapping, LocalBA, and LoopClosing
        std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N); //used in Tracking, KeyFrameDatabase, LocalMapping
        std::vector<KeyFrame*> GetCovisiblesByWeight(KeyFrame* pKF, const int &w); // used in Optimizer, MapDrawer
        int GetWeight(KeyFrame* pKFnode, KeyFrame* pKFquery); //using in Optimizer and KeyFrame implementation
        void clear(){ covis_graph.clear(); };
        bool inGraph(KeyFrame* pKF);
      //  void validateCovisiblityGraph();
        
    private:
    
        CovisibilityGraph_t covis_graph;
       // std::map<KeyFrame*, CovisNode> covis_graph;
        std::mutex graph_mutex;

  };


}//end namespace

#endif
