#include "path_searcher/astar.h"
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace Eigen;

Astar::Astar(costmap_2d::Costmap2DROS *cmap_ptr_) {

    tie_breaker_ = 1.0 + 1.0 / 1000;
    //tie_breaker_ = 1.0;
    heu_weight_ = 2;
    heu_method_ = EUCLIDEAN;
    //heu_method_ = MANHATTAN;
    //heu_method_ = DIAGONAL;

    //map
    origin_[0] =  cmap_ptr_->getCostmap()->getOriginX();
    origin_[1] =  cmap_ptr_->getCostmap()->getOriginY();
    cout << "origin_:" << origin_[0] << " " << origin_[1] << endl;
    resolution_ = cmap_ptr_->getCostmap()->getResolution();
    cout << "resolution_:" << resolution_ << endl;
    map_size_x_ = cmap_ptr_->getCostmap()->getSizeInCellsX();
    map_size_y_ = cmap_ptr_->getCostmap()->getSizeInCellsY();
    cout << "map_size: "<< map_size_x_ << " " << map_size_y_ << endl;
    Eigen::Vector2i map_size;
    map_size << map_size_x_, map_size_y_;
    Eigen::Vector2d world_size;
    world_size = indexToPos(map_size);
    world_size_x_ = world_size[0];
    world_size_y_ = world_size[1];
    cost_ =  cmap_ptr_->getCostmap()->getCharMap();
    cout << cost_[0] << endl;   //先x后y

    node_map_ = new Node * *[map_size_x_];
    for (int i = 0; i < map_size_x_; ++i) {
        node_map_[i] = new Node * [map_size_y_];

        for (int j = 0; j < map_size_y_; ++j) {
            Vector2i index;
            index << i, j;
            Vector2d position = indexToPos(index);
            node_map_[i][j] = new Node(index, position);
        }

    }
    ROS_INFO("map init finish");
    
}

Astar::~Astar() {
    for (int i = 0; i < map_size_x_; ++i) {
        for (int j = 0; j < map_size_y_; ++j) {          
            delete node_map_[i][j];
        }
    }
}

bool Astar::search(Eigen::Vector2d start, Eigen::Vector2d goal) {
    ROS_INFO("start search");
    ros::Time time1 = ros::Time::now();
    //    start << -50, -45;
    //    goal << -20,0;
    Vector2i start_index = posToIndex(start);
    Vector2d start_pos = indexToPos(start_index);
   
    ROS_INFO("start index [%i, %i]", start_index[0], start_index[1]);
    ROS_INFO("start pos [%f, %f]", start_pos[0], start_pos[1]);
    Node *cur_node = node_map_[start_index[0]][start_index[1]];
    
    cur_node->parent_ = nullptr;
    cur_node->g_score_ = 0.0;
    cur_node->f_score_ = heu_weight_ * getHeu(cur_node->position_, goal);

    Vector2i goal_index = posToIndex(goal);

    while (!open_list_.empty()) open_list_.pop(); 

    open_list_.push(cur_node);

    while (!open_list_.empty()) {
        cur_node = open_list_.top();
        //ROS_INFO("current f_score : %f", cur_node->f_score_);
        //cout << "current f_score " << cur_node->f_score_ << endl;
        //ROS_INFO("current index [%i, %i]", cur_node->index_[0], cur_node->index_[1]);
        //ROS_INFO("goal index [%i, %i]", goal_index[0], goal_index[1]);

        if (cur_node->index_ == goal_index) {

            Node *goal_node = cur_node;
            traceback(goal_node);
            break;
        }

        open_list_.pop();
        Node *neighbor_ptr;
        //expand neighbor nodes
        for (int i = -1; i < 2; ++i) {
            for (int j = -1; j < 2; ++j) {
                if (i == 0 && j == 0) continue;
                Vector2i neighbor_index;
                neighbor_index << cur_node->index_[0] + i, cur_node->index_[1] + j;
                Vector2d neughbor_pos = indexToPos(neighbor_index);
                 
                //超出地图范围，不扩展
                if (neighbor_index[0] < 0 || neighbor_index[1] < 0 || 
                    neighbor_index[0] >= map_size_x_ || neighbor_index[1] >= map_size_y_) {
                    continue;
                }

                //障碍物节点不扩展
                //这个函数没有
                //cout << cmap_ptr_->getCostmap()->getCost(neighbor_index[0], neighbor_index[1]) << endl;
                if (cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] >= 253 &&
                    cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] != costmap_2d::NO_INFORMATION) {
                    continue;
                } 

                neighbor_ptr = node_map_[neighbor_index[0]][neighbor_index[1]];
                double edge_cost = sqrt(pow(neughbor_pos[0] - cur_node->position_[0], 2) + 
                                        pow(neughbor_pos[1] - cur_node->position_[1], 2));
                //未处理过的
                if (neighbor_ptr->g_score_ == inf) {
                    neighbor_ptr->g_score_ = cur_node->g_score_ + edge_cost;
                    neighbor_ptr->f_score_ = neighbor_ptr->g_score_ + heu_weight_ * getHeu(neighbor_ptr->position_, goal);
                    neighbor_ptr->parent_ = cur_node;
                    open_list_.push(neighbor_ptr);

                } else {
                    if (neighbor_ptr->g_score_ > cur_node->g_score_ +  edge_cost) {
                        neighbor_ptr->g_score_ = cur_node->g_score_ +  edge_cost;
                        neighbor_ptr->f_score_ = neighbor_ptr->g_score_ + heu_weight_ * getHeu(neighbor_ptr->position_, goal);
                        neighbor_ptr->parent_ = cur_node;


                    } else {
                        continue;
                    }
                }
            }
        }


    }
    
    if (!open_list_.empty()) {
        ros::Time time2 = ros::Time::now();
        double time = (time2 - time1).toSec();
        ROS_WARN("Time in Astar searching is %f", time);
        return true;
    }
    return false;

}

Eigen::Vector2i Astar::posToIndex(Eigen::Vector2d position) {
    Eigen::Vector2i index1;
    index1[0] = (int)((position[0] - origin_[0]) / resolution_);
    index1[1] = (int)((position[1] - origin_[1]) / resolution_);
    return index1;
    
}
Eigen::Vector2d Astar::indexToPos(Eigen::Vector2i index) {
    Vector2d pos;
    pos[0] = origin_[0] + (index[0] + 0.5) * resolution_;
    pos[1] = origin_[1] + (index[1] + 0.5) * resolution_;
    return pos;
}
double Astar::getHeu(Eigen::Vector2d cur, Eigen::Vector2d goal) {

    double heu;
    double dx, dy, min;
    switch (heu_method_)
    {
    case EUCLIDEAN:
        heu = tie_breaker_ * (goal - cur).norm();
        cout << "欧式距离启发函数" << endl;
        break;
    case MANHATTAN:
        heu = tie_breaker_ * (fabs(goal[0] - cur[0]) + fabs(goal[1] - cur[1]));
        break;
    case DIAGONAL:
        dx = fabs(goal[0] - cur[0]);
        dy = fabs(goal[1] - cur[1]);
        min = std::min(dx, dy);
        if (dx == min) {
            heu = tie_breaker_ * (dy - dx + sqrt(2) * dx);
        } 
        if (dy == min) {
            heu = tie_breaker_ * (dx - dy + sqrt(2) * dy);
        }
        break;
    
    default:
        heu = 0;
        cout << "dijkstra" << endl;
        break;
    }

    return heu;
}

void Astar::traceback(Node *goal_node) {
    Node *cur_node = goal_node;
    path_node_.push_back(cur_node);

    while (cur_node->parent_) {
        cur_node = cur_node->parent_;
        path_node_.push_back(cur_node);
    }

    reverse(path_node_.begin(), path_node_.end());
}

std::vector<Eigen::Vector2d> Astar::getPath() {
    std::vector<Eigen::Vector2d> path;
    for (auto node : path_node_) {
        path.push_back(node->position_);
    }
    path_node_.clear(); //不断选起点和终点
    cout << "size:" << path.size() << endl;
    return path;
}
std::vector<Eigen::Vector2d> Astar::getVisitedNode() {
    std::vector<Eigen::Vector2d> node_v;
    for (int i = 0; i < map_size_x_; ++i) {
        for (int j = 0; j < map_size_y_; ++j) {
            if (node_map_[i][j]->g_score_ != inf) {
                node_v.push_back(node_map_[i][j]->position_);
            }
        }
    }
    return node_v;
}

void Astar::resetNodeMap() {
    for (int i = 0; i < map_size_x_; ++i) {
        for (int j = 0; j < map_size_y_; ++j) {
            
            node_map_[i][j]->g_score_ = inf;
            node_map_[i][j]->f_score_ = 0;
            node_map_[i][j]->parent_ = nullptr;
        }

    }
}