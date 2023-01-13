/**
 * 全局路径导航起止点坐标发布节点
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

int main(int argc, char **argv) {
    // ROS节点初始化
    ros::init(argc, argv, "state_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 读取话题参数
    std::string start_topic, goal_topic;
    nh.param<std::string>("topics/start_topic", start_topic, "/gbpl_state/start");
    nh.param<std::string>("topics/goal_topic", goal_topic, "/gbpl_state/goal");

    // 读取起止点目标信息
    double start_position_x, start_position_y, start_position_z, start_yaw;
    double goal_position_x, goal_position_y, goal_position_z, goal_yaw;
    nh.param<double>("state_publisher/start_position_x", start_position_x, 0);
    nh.param<double>("state_publisher/start_position_y", start_position_y, 0);
    nh.param<double>("state_publisher/start_position_z", start_position_z, 0);
    nh.param<double>("state_publisher/start_yaw", start_yaw, 0);
    nh.param<double>("state_publisher/goal_position_x", goal_position_x, 0);
    nh.param<double>("state_publisher/goal_position_y", goal_position_y, 0);
    nh.param<double>("state_publisher/goal_position_z", goal_position_z, 0);
    nh.param<double>("state_publisher/goal_yaw", goal_yaw, 0);

    // 计算四元数
    geometry_msgs::Quaternion start_orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, start_yaw); // 参数为 r, p, y
    geometry_msgs::Quaternion goal_orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);

    // 创建发布器
    ros::Publisher start_pub = nh.advertise<geometry_msgs::PoseStamped>(start_topic, 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        // 初始化起点与终点消息
        geometry_msgs::PoseStamped start_msg, goal_msg;
        start_msg.header.stamp = ros::Time::now();
        start_msg.header.frame_id = "map";
        start_msg.pose.position.x = start_position_x;
        start_msg.pose.position.y = start_position_y;
        start_msg.pose.position.z = start_position_z;
        start_msg.pose.orientation = start_orientation;


        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = goal_position_x;
        goal_msg.pose.position.y = goal_position_y;
        goal_msg.pose.position.z = goal_position_z;
        goal_msg.pose.orientation = goal_orientation;

        // 发布消息
		start_pub.publish(start_msg);
		goal_pub.publish(goal_msg);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
