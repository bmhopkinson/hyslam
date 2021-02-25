#include <SpanningTree.h>
#include <algorithm>

namespace ORB_SLAM2 {
    /// SPANNING TREE NODE FUNCTIONS ///////////
    SpanningTreeNode::SpanningTreeNode(){}

    SpanningTreeNode::SpanningTreeNode(KeyFrame* pKF){
        unique_lock<std::mutex> lock(node_mutex);
        pKF_node = pKF;
    }

    void SpanningTreeNode::setParent(KeyFrame* pKF){
        unique_lock<std::mutex> lock(node_mutex);
        parent = pKF;
        pKF_node->setParent(parent);
    }

    void SpanningTreeNode::addChild(KeyFrame * pKF){
        unique_lock<std::mutex> lock(node_mutex);
        children.insert(pKF);
    }

    void SpanningTreeNode::eraseChild(KeyFrame * pKF){
        unique_lock<std::mutex> lock(node_mutex);
        children.erase(pKF);
    }

    std::set<KeyFrame*> SpanningTreeNode::getChildren(){
        unique_lock<std::mutex> lock(node_mutex);
        return children;
    }

    bool SpanningTreeNode::isChild(KeyFrame * pKF){
        unique_lock<std::mutex> lock(node_mutex);
        return children.count(pKF);
    }

    /// SPANNING TREE FUNCTIONS ///////////

    bool SpanningTree::inTree(KeyFrame* pKF){
        return spanning_tree.count(pKF);
    }
    int SpanningTree::addNode(KeyFrame* pKF){
        if(inTree(pKF)){
            return -1;
        }
        else{
            {
                unique_lock<std::mutex> lock(tree_mutex);
                spanning_tree.insert({pKF, std::make_unique<SpanningTreeNode>(pKF)});
            }
            return 0;
        }
    }

    int SpanningTree::addNode(KeyFrame* pKF_node, KeyFrame* parent){
        if(inTree(pKF_node)){
            return -1;
        }
        else{
            {
                unique_lock<std::mutex> lock(tree_mutex);
                spanning_tree.insert({pKF_node, std::make_unique<SpanningTreeNode>(pKF_node)});
                spanning_tree[pKF_node]->setParent(parent);

                if( inTree(parent) ) { //KeyFrame 0 has no parent so this test is necessary
                    spanning_tree[parent]->addChild(pKF_node);
                }
            }
            return 0;
        }
    }
    int SpanningTree::eraseNode(KeyFrame* pKF) {
        if(!inTree(pKF)){
            return -1;
        }
        {
            unique_lock<std::mutex> lock(tree_mutex);
            KeyFrame* parent = spanning_tree[pKF]->getParent();
            if(inTree(parent)){
                spanning_tree[parent]->eraseChild(pKF);
            }
            spanning_tree.erase(pKF);
        }
        return 0;
    }

    KeyFrame* SpanningTree::getParent(KeyFrame* pKF){
        if(!inTree(pKF)){
            return nullptr;
        }
       return spanning_tree[pKF]->getParent();
    }

    std::set<KeyFrame*> SpanningTree::getChildren(KeyFrame* pKF){
        if(!inTree(pKF)){
            return std::set<KeyFrame*>();
        }
        return spanning_tree[pKF]->getChildren();

    }

    bool SpanningTree::isChild(KeyFrame* pKF_node, KeyFrame* pKF_query){
        if(!inTree(pKF_node)){
            return false;
        }
        return spanning_tree[pKF_node]->isChild(pKF_query);
    }

    int SpanningTree::changeParent(KeyFrame* pKF_node, KeyFrame* new_parent){
        if(!inTree(pKF_node)){
            return -1;
        }
        unique_lock<std::mutex> lock(tree_mutex);
        KeyFrame* old_parent = spanning_tree[pKF_node]->getParent();
        spanning_tree[pKF_node]->setParent(new_parent);
        spanning_tree[old_parent]->eraseChild(pKF_node);
        spanning_tree[new_parent]->addChild(pKF_node);

        return 0;
    }
/*
    void SpanningTree::validateSpanningTree(){
        int n_errors = 0;
        int n_correct = 0;
        for(auto it = spanning_tree.begin(); it != spanning_tree.end(); ++it){
            KeyFrame* pKF = it->first;
            std::set<KeyFrame*> children_pKF = pKF->GetChilds();
            std::set<KeyFrame*> children_ST = spanning_tree[pKF]->getChildren();

            std::vector<KeyFrame*> vchildren_pKF(children_pKF.begin(), children_pKF.end());
            std::vector<KeyFrame*> vchildren_ST(children_ST.begin()  , children_ST.end());
            std::sort(vchildren_pKF.begin(), vchildren_pKF.end(), [](const KeyFrame* a, const KeyFrame*b) -> bool {return a->mnId < b->mnId; } );
            std::sort(vchildren_ST.begin() , vchildren_ST.end() , [](const KeyFrame* a, const KeyFrame*b) -> bool {return a->mnId < b->mnId; } );
            std::vector<KeyFrame*> diff;

            std::set_difference(vchildren_pKF.begin(), vchildren_pKF.end(),
                                           vchildren_ST.begin(), vchildren_ST.end(), diff.begin());

            std::cout << "child differences: " <<diff.size() <<std::endl;
            n_errors += diff.size();
            n_correct += std::max( vchildren_pKF.size(), vchildren_ST.size() )  - diff.size();

            std::cout << "VALIDATE KEYFRAME: " << pKF->mnId << std::endl;
            if(pKF->GetParent()) {
                std::cout << "KeyFrame data, parent: " << pKF->GetParent()->mnId << std::endl;
            }
            std::cout << "children: " ;
            for(auto kfit = children_pKF.begin(); kfit != children_pKF.end(); ++kfit){
                std::cout << (*kfit)->mnId << "\t";
            }
            if(pKF->GetParent()) {
                std::cout << std::endl << "Spanning Tree data, parent: " << spanning_tree[pKF]->getParent()->mnId
                          << std::endl << "children: ";
                if(pKF->GetParent()->mnId != spanning_tree[pKF]->getParent()->mnId){
                    std::cout << "parents disagree" << std::endl;
                    n_errors++;
                } else {n_correct++;}
            }
            for(auto stit = children_ST.begin(); stit != children_ST.end(); ++stit){
                std::cout << (*stit)->mnId << "\t";
            }
            std::cout << std::endl;

        }
        std::cout << "FINISHED VALIDATING SPANNING TREE, n_correct: " << n_correct << " , n_errors: " << n_errors << std::endl;
    }

*/

} // close namespace