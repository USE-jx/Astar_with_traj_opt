#include "simple_optimal_control/point_motion.h"
#include "ros/ros.h"
#include "simple_optimal_control/matplotlibcpp.h"
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_listener.h>
#include "path_searcher/astar.h"

using namespace std;
using namespace casadi;

/*---------plot related-------------*/
namespace plt = matplotlibcpp;

double t_plot = 0;

vector<double> x_vec;
vector<double> y_vec;
vector<double> v_vec;
vector<double> a_vec;
vector<double> time_vec;
int idx = 0;


/*---------A* related-------------*/
ros::Publisher path_vis_pub, visited_nodes_vis_pub;

Eigen::Vector2d start_astar, goal_astar;
std::unique_ptr<Astar> astar_search;
std::vector<Eigen::Vector2d> path_v;

void visPath(std::vector<Eigen::Vector2d> path);
void visVisitedNode();


/*---------optimize related-------------*/
ros::Publisher spherePub, obs_polygon_pub, path_pub;

nav_msgs::Path path;
geometry_msgs::PoseStamped pose;
DM sol_state, sol_control, sol_T;
int N = 100;
double dt = 0.01;

bool has_start = false, has_goal = false;
Eigen::Vector3d start, goal;

std::unique_ptr<PointMotion> point_motion_ptr;

/*-------------循环等待rviz的起点和终点--------------*/
void run() {
    //ROS_INFO("run()");
    if (has_start && has_goal) {
        ROS_INFO("has start and goal");
        
        ros::Time time1 = ros::Time::now();
        if (astar_search->search(start_astar, goal_astar)) {
            ROS_INFO("search success");
            path_v = astar_search->getPath();
            visPath(path_v);
            visVisitedNode();
            point_motion_ptr->setInitial(path_v);
            path_v.clear();
            astar_search->resetNodeMap();

            cout << "start optimize" << endl;
            ros::Time time2 = ros::Time::now();
            if (point_motion_ptr->solve(start, goal)) {
                cout << "solve success" << endl;

                point_motion_ptr->getSolution(sol_state, sol_control, sol_T);

                cout << "state:\n" << sol_state << "\ncontrol:\n" << sol_control << "\nT:\n" << sol_T << "\n\n";
                
                ros::Time time3 = ros::Time::now();
                double time_op = (time2 - time1).toSec();
                double time_sum = (time3 - time1).toSec();
                
                ROS_WARN("Time in A* is %f", time_op);
                ROS_WARN("Time in A*+optimize is %f", time_sum);

                dt = static_cast<double>(sol_T / N);

                /*----------画图--------------*/
                for (int i = 0; i < N; ++i) {
                    x_vec.push_back(static_cast<double>(sol_state(0, i)));
                    y_vec.push_back(static_cast<double>(sol_state(1, i)));
                    
                    v_vec.push_back(static_cast<double>(sol_state(3, i)));
                    if (i < N - 1) {
                        a_vec.push_back(static_cast<double>(sol_control(0, i)));
                    }
                    time_vec.push_back(i * dt);
                }
                a_vec.push_back(a_vec[N-2]);
                
                path.header.frame_id = "odom";
                for (int i = 0; i < x_vec.size(); ++i) {
                    pose.pose.position.x = x_vec[i];
                    pose.pose.position.y = y_vec[i];
                    path.poses.push_back(pose);
                }

                path_pub.publish(path);
                path.poses.clear();

                plt::subplot(2,1,1);
                plt::title("motion speed");
                plt::plot(time_vec, v_vec, "b-");
                
                plt::subplot(2,1,2);
                plt::title("accelerate");
                plt::plot(time_vec, a_vec, "g-");

                plt::show();

                v_vec.clear();
                a_vec.clear();
                time_vec.clear();
                has_start = false;
                has_goal = false;
            }
        } else {
            ROS_WARN("search fail");
            has_start = false;
            has_goal = false;
        }
    } else {
        ROS_INFO("waiting start or goal");
    }



}

/*-------------A* 可视化函数----------------------------*/

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


/*-------------optimal control---------------*/

void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {

    start[0] = msg.pose.pose.position.x;
    start[1] = msg.pose.pose.position.y;
    start[2] = tf2::getYaw(msg.pose.pose.orientation);
    start_astar[0] = msg.pose.pose.position.x;
    start_astar[1] = msg.pose.pose.position.y;
    has_start = true;
    ROS_INFO("receive start position");

}

void goalCallback(const geometry_msgs::PoseStamped msg) {

    goal[0] = msg.pose.position.x;
    goal[1] = msg.pose.position.y;
    goal[2] = tf2::getYaw(msg.pose.orientation);
    goal_astar[0] = msg.pose.position.x;
    goal_astar[1] = msg.pose.position.y;
    has_goal = true;
    ROS_INFO("receive goal position");
}

void visualizeObstacle() {
    // 创建一个Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom"; // 设置坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "polygon_namespace"; // 命名空间
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP; // 使用线条来表示多边形
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // 设置线条颜色和宽度
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.1; // 线宽

    // 添加多边形的顶点
    geometry_msgs::Point p1, p2, p3;
    p1.x = 4.1;
    p1.y = -10.5;
    p1.z = 0.0;

    p2.x = -6.5;
    p2.y = -7.6;
    p2.z = 0.0;

    p3.x = -6.5;
    p3.y = -10.5;
    p3.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p1); // 将第一个顶点添加到末尾以封闭多边形

    obs_polygon_pub.publish(marker);

}

void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }


//可视化小红球
void vis_points(const ros::TimerEvent &e) {
    if (x_vec.empty()) return;
    visualizeSphere({x_vec[idx], y_vec[idx], 0}, 0.2);

    if (idx < x_vec.size()) {
        idx++;
    } else {
        x_vec.clear();
        y_vec.clear();

        idx = 0;
    }
    //cout << idx << endl;
    
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoints_publisher");
    ros::NodeHandle nh;

    spherePub = nh.advertise<visualization_msgs::Marker>("/spheres", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
    obs_polygon_pub = nh.advertise<visualization_msgs::Marker>("/obs_polygon", 1);

    path_vis_pub = nh.advertise<nav_msgs::Path>("/astar_path", 1);
    visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("/visited_nodes", 1);

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
    
    ros::Timer vis_point; 

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer); //build a costmap

    point_motion_ptr.reset(new PointMotion());
    astar_search.reset(new Astar(&lcr));


    vis_point = nh.createTimer(ros::Duration(dt), vis_points);


    ros::Rate r(10);

    while (ros::ok()) {
        
        visualizeObstacle();
        run();

        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}