#include "global_body_planner/rviz_interface.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

RVizInterface::RVizInterface(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server	从参数服务器加载参数
	std::string body_plan_topic, body_plan_viz_topic, discrete_body_plan_topic, discrete_body_plan_viz_topic, footstep_plan_viz_topic;

	nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");	// 插值后的规划消息订阅话题
	nh.param<std::string>("topics/visualization/body_plan", body_plan_viz_topic, "/visualization/body_plan");	// 插值后的rviz规划消息发布话题

	nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");	// 未插值的离散状态规划消息订阅话题
	nh.param<std::string>("topics/visualization/discrete_body_plan", discrete_body_plan_viz_topic, "/visualization/discrete_body_plan");	// 未插值的离散rviz状态规划消息发布话题

	nh.param<std::string>("map_frame", map_frame_, "/map");	// map frame
	nh.param<double>("visualization/update_rate", update_rate_, 10); // 收发数据更新频率

	// Setup pubs and subs
	body_plan_sub_ = nh_.subscribe(body_plan_topic, 1, &RVizInterface::bodyPlanCallback, this);	// 插值后的规划消息订阅者
	discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic, 1, &RVizInterface::discreteBodyPlanCallback, this);	// 未插值的离散状态规划消息订阅者
	body_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(body_plan_viz_topic, 1);	// 插值后的rviz规划消息发布者
	discrete_body_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(discrete_body_plan_viz_topic, 1);	// 未插值的离散rviz状态规划消息发布者
}

// 插值后的规划消息订阅者回调函数，将规划消息转换为 nav_msgs::Path 消息并发布
void RVizInterface::bodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr &msg) {
	// Initialize Path message to visualize body plan	初始化Path消息以可视化规划
	nav_msgs::Path body_plan_viz;
	body_plan_viz.header = msg->header;

	// Loop through the BodyPlan message to get the state info and add to private vector
	// 遍历接收到的 BodyPlan 消息，并将状态添加到 Path 消息中
	int length = msg->states.size();	// 状态数目
	for (int i = 0; i < length; i++) {

		// Load in the pose data directly from the Odometry message	直接从里程计消息中加载姿态数据
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header = msg->states[i].header;
		pose_stamped.pose = msg->states[i].pose.pose;

		// Add to the path message	添加到 Path 消息
		body_plan_viz.poses.push_back(pose_stamped);
	}

	// Publish the full path	发布所有路径消息
	body_plan_viz_pub_.publish(body_plan_viz);
}

// 未插值的离散状态规划消息订阅者回调函数，将规划消息转换为 visualization_msgs::Marker 消息并发布
void RVizInterface::discreteBodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr &msg) {

	// Construct Marker message	构建标记信息
	visualization_msgs::Marker discrete_body_plan;

	// Initialize the headers and types	初始化headers和类型
	discrete_body_plan.header = msg->header;
	discrete_body_plan.id = 0;
	discrete_body_plan.type = visualization_msgs::Marker::POINTS;

	// Define the shape of the discrete states	定义离散状态的形状
	double scale = 0.2;
	discrete_body_plan.scale.x = scale;
	discrete_body_plan.scale.y = scale;
	discrete_body_plan.scale.z = scale;
	discrete_body_plan.color.r = 0.733f;
	discrete_body_plan.color.a = 1.0;	// 透明度，越大越不透明

	// Loop through the discrete states	遍历离散状态
	int length = msg->states.size();
	for (int i = 0; i < length; i++) {
		geometry_msgs::Point p;
		p.x = msg->states[i].pose.pose.position.x;
		p.y = msg->states[i].pose.pose.position.y;
		p.z = msg->states[i].pose.pose.position.z;
		discrete_body_plan.points.push_back(p);
	}

	// 发布离散状态
	discrete_body_plan_viz_pub_.publish(discrete_body_plan);
}

void RVizInterface::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		// Collect new messages on subscriber topics
		ros::spinOnce();

		// Enforce update rate
		// r.sleep();
	}
}
