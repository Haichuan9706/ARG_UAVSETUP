#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "min_snap/minco.h"

ros::Publisher goal_list_pub;
ros::Publisher poly_coef_pub;
ros::Subscriber rviz_goal_sub;
ros::Subscriber odom_sub;

int id = 0;
double mean_vel;
const int GOAL_HEIGHT = 1;
nav_msgs::Odometry odom;
geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;
my_planner::Minco minco_solver;
std::vector<Eigen::Vector3d> waypoints;
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;

void pub_poly_coefs()
{
    //std::cout << "pub_poly_coefs 0" << std::endl;

    Eigen::MatrixXd poly_coef = minco_solver.getPolyCoef();
    Eigen::VectorXd time = minco_solver.getTime();
    //std::cout << "pub_poly_coefs 1:" << poly_coef.rows() << " " << poly_coef.cols() << std::endl;

    poly_pub_topic.num_segment = goal_list.poses.size() - 1;
    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id = id;

    // display decision variable
    ROS_WARN("decision variable:");

    for (int i = 0; i < time.size(); i++)
    {
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
        {
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }
    //std::cout << "pub_poly_coefs 2" << std::endl;
    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();

    poly_coef_pub.publish(poly_pub_topic);
}

void solve_min_snap()
{
    Eigen::Vector3d wp;
    waypoints.clear();
    for (int i = 0; i < int(goal_list.poses.size()); i++)
    {
        wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
        waypoints.push_back(wp);
    }
    minco_solver.Init(waypoints, mean_vel);
    ROS_INFO("Init success");
    minco_solver.MincoConstruct();
    pub_poly_coefs();
}

void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pt = msg->pose;
    if (goal_pt.position.z < 0)
    {
        goal_pt.position.z = GOAL_HEIGHT;
        goal_list.poses.push_back(goal_pt);
        goal_pt.position = odom.pose.pose.position;
        goal_list.poses.insert(goal_list.poses.begin(), goal_pt);
        goal_list.header.stamp = ros::Time::now();
        goal_list.header.frame_id = "world";
        goal_list.header.seq = id++;
        goal_list_pub.publish(goal_list);
        //std::cout << "rviz_goal_cb 0" << std::endl;
        solve_min_snap();
        ROS_INFO("solver finished");
        goal_list.poses.clear();
    }
    else
    {
        goal_pt.position.z = GOAL_HEIGHT;
        goal_list.poses.push_back(goal_pt);
    }
}

void odom_goal_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minco_generator");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    odom_sub = nh.subscribe("/odom_topic", 10, odom_goal_cb);
    rviz_goal_sub = nh.subscribe("/rviz_goal", 10, rviz_goal_cb);
    goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);
    poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);

    poly_pub_topic.num_order = 7;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);

    ROS_INFO("minco_generator start");
    ros::param::get("/minco_generator/meanvel", mean_vel);
    double mean_vel_last = mean_vel;
    while (ros::ok())
    {
        if (ros::param::get("/minco_generator/meanvel", mean_vel) && mean_vel != mean_vel_last)
        {
            std::cout << "new speed set: " << mean_vel << std::endl;
            mean_vel_last = mean_vel;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}