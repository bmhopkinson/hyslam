#ifndef SPANNINGTREE_H_
#define SPANNINGTREE_H_

#include <KeyFrame.h>
#include <set>
#include <mutex>


//NEED TO RETHINK SPANNING TREE - not really clear what it means when a single map can have disjointed segments (e.g. monocular for imaging) - perhaps they shouldn't and should always initilalize a new map
// lack of parent leads to some segfaults when parent is null - have put checks in some locations to avoid this, but really it doesn't make sense to have a spanning tree in which multiple members don't have a parent
namespace HYSLAM{
    class SpanningTreeNode{
    public:
        SpanningTreeNode();
        SpanningTreeNode(KeyFrame* pKF);
        void setParent(KeyFrame* pKF);
        KeyFrame* getParent() { return parent; }
        void addChild(KeyFrame* pKF);
        void eraseChild(KeyFrame* pKF);
        std::set<KeyFrame*> getChildren();
        bool isChild(KeyFrame* pKF);

    private:
        KeyFrame* pKF_node;
        KeyFrame* parent;
        std::set<KeyFrame*> children;
        std::mutex node_mutex;  //would it make sense ot have this public and then the SpanningTree could lock it when doing multiple operations on a single node? rather than locking at each function
    };

    class SpanningTree{
    public:
        using SpanningTree_t = std::map< KeyFrame*, std::unique_ptr<SpanningTreeNode> >;
        int addNode(KeyFrame* pKF);
        int addNode(KeyFrame* pKF_node, KeyFrame* parent);
        int eraseNode(KeyFrame* pKF);  //removes the parent's link to pKF and deletes pKF in spanning tree, but does not redistribute children b/c that requires collaboration w/ covis graph
        void clear() { spanning_tree.clear(); }
        KeyFrame* getParent(KeyFrame* pKF);
        std::set<KeyFrame*> getChildren(KeyFrame* pKF);
        bool isChild(KeyFrame* pKF_node, KeyFrame* pKF_query);
        int changeParent(KeyFrame* pKF_node, KeyFrame* new_parent);
        int handleSetBad(KeyFrame* pKF_bad);
       // void validateSpanningTree();

    private:
        SpanningTree_t spanning_tree;
        bool inTree(KeyFrame* pKF);
        std::mutex tree_mutex;
    };

} //close namespace

#endif
