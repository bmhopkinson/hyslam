#ifndef SPANNINGTREE_H_
#define SPANNINGTREE_H_

/*
 * minimal graph connecting all KeyFrame
 * not logically compatible with the current map structure that allows disjointed sections of a map so it used in a hacky way right now (and i think responsible for many of the segfaults) - preferred solution is to change map so that it's a recursive data structure with a spanning tree per node
 *
 * SpanningTreeNode:
 *  data:
 *      pKF_node - keyframe the node represents
 *      parent - parent in the tree
 *      children - children in the tree
 *
*   functionality is all pretty self explantory the complicated work of setting new parents when a keyframe is destroyed is done by KeyFrameDB
 *
 *
 *
 *
 * //NEED TO RETHINK SPANNING TREE - not really clear what it means when a single map can have disjointed segments (e.g. monocular for imaging) - perhaps they shouldn't and should always initilalize a new map
// lack of parent leads to some segfaults when parent is null - have put checks in some locations to avoid this, but really it doesn't make sense to have a spanning tree in which multiple members don't have a parent
 */

#include <KeyFrame.h>
#include <set>
#include <mutex>



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
        std::mutex node_mutex;
    };

    class SpanningTree{
    public:
        SpanningTree();
        using SpanningTree_t = std::map< KeyFrame*, std::unique_ptr<SpanningTreeNode> >;
        bool exists(KeyFrame* pKF);
        int addNode(KeyFrame* pKF);
        int addNode(KeyFrame* pKF_node, KeyFrame* parent);
        int eraseNode(KeyFrame* pKF);  //removes the parent's link to pKF and deletes pKF in spanning tree, but does not redistribute children b/c that requires collaboration w/ covis graph
        KeyFrame* parentForNewKeyFrame();
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
        std::list<KeyFrame*> next_parent;
    };

} //close namespace

#endif
