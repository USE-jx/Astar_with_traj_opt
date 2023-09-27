#include "path_searcher/astar.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

using namespace std;

ros::Publisher path_vis_pub, visited_nodes_vis_pub;

bool has_start, has_goal;
Eigen::Vector2d start, goal;
std::unique_ptr<Astar> astar_search;
std::vector<Eigen::Vector2d> path_v;


void visPath(std::vector<Eigen::Vector2d> path) {
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "odom";
    nav_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pos;
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];
        nav_path.poses.push_back(pos);
    }
    path_vis_pub.publish(nav_path);

}

void visVisitedNode() {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "odom";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "visited_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = astar_search->resolution_;
    node_vis.scale.y = astar_search->resolution_;

    geometry_msgs::Point pt;
    std::vector<Eigen::Vector2d>  nodes = astar_search->getVisitedNode();
    for (int i = 0; i < nodes.size(); ++i) {
        Eigen::Vector2d pos = nodes[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = 0;
        node_vis.points.push_back(pt);
    }
    visited_nodes_vis_pub.publish(node_vis);
}
void run() {
    //ROS_INFO("run()");
    if (has_start && has_goal) {
        ROS_INFO("has start and goal");
        if (astar_search->search(start, goal)) {
            ROS_INFO("search success");
            path_v = astar_search->getPath();
            visPath(path_v);
            visVisitedNode();
            path_v.clear();
            astar_search->resetNodeMap();
            has_start = false;
            has_goal = false;
            
            
        } else {
            ROS_WARN("search fail");
        }
    } else {
        ROS_INFO("waiting start or goal");
    }

}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {

    start[0] = msg.pose.pose.position.x;
    start[1] = msg.pose.pose.position.y;
    has_start = true;
    ROS_INFO("receive start position");

}

void goalCallback(const geometry_msgs::PoseStamped msg) {

    goal[0] = msg.pose.position.x;
    goal[1] = msg.pose.position.y;
    has_goal = true;
    ROS_INFO("receive goal position");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh;

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);

    path_vis_pub = nh.advertise<nav_msgs::Path>("/astar_path", 1);
    visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("/visited_nodes", 1);
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer); //build a costmap
    astar_search.reset(new Astar(&lcr));
    
    ros::Rate r(10);

    while (ros::ok()) {
        run();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}