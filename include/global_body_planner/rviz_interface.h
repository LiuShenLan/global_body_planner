#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <ros/ros.h>
#include <global_body_planner/BodyPlan.h>

//! A class for interfacing between RViz and planning topics.	用于连接rviz和规划话题的类
/*!
	RVizInterface is a container for all of the logic utilized in the template node.
	The implementation must provide a clean and high level interface to the core algorithm
	RVizInterface是模板节点中利用的所有逻辑的容器。该实现必须为核心算法提供一个干净的、高水平的接口。
*/
class RVizInterface {
public:
	/**
	 * @brief Constructor for RVizInterface Class	构造函数
	 * @param[in] nh ROS NodeHandle to publish and subscribe from	ros句柄
	 * @return Constructed object of type RVizInterface
	 */
	RVizInterface(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
     * @brief Callback function to handle new body plan data	插值后的规划消息订阅者回调函数，将规划消息转换为 nav_msgs::Path 消息并发布
     * @param[in] Body plan message contining interpolated output of body planner
     */
	void bodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr &msg);

	/**
	 * @brief Callback function to handle new body plan discrete state data	未插值的离散状态规划消息订阅者回调函数，将规划消息转换为 visualization_msgs::Marker 消息并发布
	 * @param[in] Body plan message contining discrete output of body planner
	 */
	void discreteBodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr &msg);

	/// ROS subscriber for the body plan	插值后的规划消息订阅者
	ros::Subscriber body_plan_sub_;

	/// ROS subscriber for the body plan	未插值的离散状态规划消息订阅者
	ros::Subscriber discrete_body_plan_sub_;

	/// ROS Publisher for the interpolated body plan vizualization	插值后的rviz规划消息发布者
	ros::Publisher body_plan_viz_pub_;

	/// ROS Publisher for the discrete body plan vizualization	未插值的离散rviz状态规划消息发布者
	ros::Publisher discrete_body_plan_viz_pub_;

	/// Nodehandle to pub to and sub from	ros节点句柄
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data, unused since pubs are called in callbacks	收发数据频率，因为回调函数中发布消息，所以不再使用
	double update_rate_;

	/// Handle for the map frame
	std::string map_frame_;
};

#endif // RVIZ_INTERFACE_H
