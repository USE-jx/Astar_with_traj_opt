#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <queue>
#include <costmap_2d/costmap_2d_ros.h>

#define inf 1 << 20


class Node {
public:
    Eigen::Vector2i index_;     //地图索引
    Eigen::Vector2d position_;  //世界系位置
    double g_score_, f_score_;  
    Node *parent_;              //父节点用于回溯

    Node(Eigen::Vector2i index, Eigen::Vector2d position) {
        index_ = index;
        position_ = position;
        parent_ = nullptr;
        g_score_ = inf;
        f_score_ = 0;
        //待补充
    }
    ~Node() {}

};

//为优先级队列定义的仿函数，比较节点的f_score
struct compare {
    bool operator() (const Node *node1, const Node *node2) const {
        return node1->f_score_ > node2->f_score_;
    }
};
class Astar {
private:

    std::priority_queue<Node*, std::vector<Node *>, compare> open_list_;
    std::vector<Node *>  path_node_;
    std::vector<Eigen::Vector2d> path_;
    std::vector<Node *> visited_node_; 

    //map params
    Node * **node_map_;
    costmap_2d::Costmap2DROS *cmap_ptr_;
    unsigned char* cost_;
    double tie_breaker_; 
    int heu_method_;
    double heu_weight_;
    enum {EUCLIDEAN, MANHATTAN, DIAGONAL}; //三种启发式函数
public:
    //map params
    Eigen::Vector2d origin_; //地图原点
    double resolution_;      //分辨率
    uint map_size_x_, map_size_y_;      //地图大小
    double world_size_x_, world_size_y_;    

    Astar(costmap_2d::Costmap2DROS *cmap_ptr_);
    ~Astar();

    bool search(Eigen::Vector2d start, Eigen::Vector2d goal); 
    Eigen::Vector2i posToIndex(Eigen::Vector2d position);  //世界系位置转地图栅格索引
    Eigen::Vector2d indexToPos(Eigen::Vector2i index);      //地图栅格索引转世界系位置
    double getHeu(Eigen::Vector2d cur, Eigen::Vector2d goal);
    void traceback(Node *goal_node);    //回溯得到整条路径上的node
    std::vector<Eigen::Vector2d> getPath();
    std::vector<Eigen::Vector2d> getVisitedNode();
    void resetNodeMap();
};

#endif