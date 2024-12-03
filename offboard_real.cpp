#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>

// 全局变量存储无人机状态和VICON数据
mavros_msgs::State current_state;
geometry_msgs::PoseStamped vicon_pose;

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// VICON数据回调函数
void vicon_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    // 将TransformStamped转换为PoseStamped
    vicon_pose.header = msg->header;
    vicon_pose.pose.position.x = msg->transform.translation.x;
    vicon_pose.pose.position.y = msg->transform.translation.y;
    vicon_pose.pose.position.z = msg->transform.translation.z;

    // 使用VICON提供的旋转数据
    vicon_pose.pose.orientation = msg->transform.rotation;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offboard_hover");
    ros::NodeHandle nh;

    // 订阅无人机状态和VICON数据
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/drone/drone", 10, vicon_cb);

    // 发布目标位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    ros::Rate rate(20);
    
    // 等待FCU连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 主循环：持续发布VICON位置数据
    while(ros::ok()){
        // 检查当前是否ARM
        if(current_state.armed){
            // 只在ARM状态下发布位置
            local_pos_pub.publish(vicon_pose);
        } else {
            ROS_WARN_THROTTLE(1, "Drone is disarmed, stopping position updates");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
