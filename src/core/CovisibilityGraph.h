#ifndef COVISIBILITYGRAPH_H_
#define COVISIBILITYGRAPH_H_

/*
 * graph recording covisibility of landmarks between keyframes
 * graph nodes = keyframes; edges: weights = landmarks shared between keyframes; edges are added if weight exceeds a threshold (currently hardcoded at 15)
 *
 *
 * CovisNode functionality
 *      AddConnection, EraseConnection directly add to mConnectedKeyFrameWeights; these functions call UpdateBestCovisibles to update mvpOrderedConnectedKeyFrames and mvOrderedWeights
 *      UpdateConnnections(symmetric_updates) works through all landmarks viewed in node_KF and finds covisible keyframes updating all node datastructures- returns symmetric_updates for updating covisibility nodes of connected KFs
 *      UpdateBestCovisibles - used internally to update mvpOrderedConnectedKeyFrames and mvOrderedWeights based on content of mConnectedKeyFrameWeights
 *      both GetConnectedKeyFrames() and GetVectorCovisibleKeyFrames() return all connected KFs, the first function as a set, the second function as a vector
 *      GetBestCovisibilityKeyFrames(const int &N) returns N keyframes with highest edge weights (or all keyframes)
 *      GetCovisiblesByWeight(const int &w) returns all keyframes with edge weight exceeding w.
 *
 * CovisibilityGraph functionality
 *       addNode(KeyFrame* pKF) - adds pKF to the graph, creating a CovisNode for it and calling CovisibilityGraph::UpdateConnections(pKF) on it
 *       eraseNode(KeyFrame* pKF) - first erases all edges containing pKF in CovisibilityGraph then deletes the node for pKF
 *       UpdateConnections(KeyFrame* pKF); updates covis connections for pKF (by calling CovisNode::UpdateConnections()) and handles updating weights to keyframes connected to pKF
 *       remaining functions (GetConnectedKeyFrames through GetWeight simply forward calls on to respective CovisNode function)
 */

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
        std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;//connected keyframes ordered from most shared landmarks to least
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
        int eraseConnection(KeyFrame* pKFnode, KeyFrame* pKFconn);
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
