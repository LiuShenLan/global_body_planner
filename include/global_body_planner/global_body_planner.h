#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <global_body_planner/BodyPlan.h>
#include "global_body_planner/fast_terrain_map.h"

#include "global_body_planner/planning_utils.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_star_connect.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>


using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global body planning node.
   This algorithm requires an height map of the terrain as a GridMap message type, and will publish
   the global body plan as a BodyPlan message over a topic. It will also publish the discrete states
   used by the planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner {
public:
	/**
	 * @brief Constructor for GlobalBodyPlanner Class 构造函数
	 * @param[in] nh Node handle
	 * @return Constructed object of type GlobalBodyPlanner
	 */
	GlobalBodyPlanner(ros::NodeHandle nh);

	/**
	 * @brief Call the correct planning class and compute statistics  调用正确的规划类并计算统计信息
	 */
	void callPlanner();

	/**
	 * @brief Primary work function in class, called in node file for this component  类中的主要工作函数，在该组件的节点文件中调用
	 */
	void spin();


private:
	/**
	 * @brief Callback function to handle new terrain map data  订阅到新的地图信息时更新地图数据
	 * @param[in] msg the message contining map data
	 */
	void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);

	/**
	 * @brief 设置规划器的参数
	 * @param[out] rrt_obj 要设置的规划器
	 */
	void setPlannerParameter(RRTClass& rrt_obj);

	/**
	 * @brief Set the start and goal states of the planner  设置规划器的起止点
	 */
	void setStartAndGoalStates();

	/**
	 * @brief Clear the plan member variables 清除规划变量
	 */
	void clearPlan();

	/**
	 * @brief Update the body plan with the current plan  将指定状态添加到规划信息中
	 * @param[in] t Time of state in trajectory 指定状态的时间戳
	 * @param[in] body_state Body state			指定状态
	 * @param[out] body_plan_msg Body plan message	输出：规划消息
	 */
	void addBodyStateToMsg(double t, State body_state, global_body_planner::BodyPlan &body_plan_msg);

	/**
	 * @brief 发布在规划过程中遍历到的所有的状态
	 * @param[in] allStatePosition 在规划过程中遍历到的所有的状态
	 */
	void publishAllState();

	/**
	 * @brief Publish the current body plan   将状态规划转换为ros消息并发布
	 */
	void publishPlan();

	/**
	 * @brief Wait until a map message has been received and processed  等待收到并处理地图消息
	 */
	void waitForMap();

	/// Subscriber for terrain map messages 高程图消息订阅者
	ros::Subscriber terrain_map_sub_;

	/// Publisher for body plan messages    插值后的规划消息发布者
	ros::Publisher body_plan_pub_;

	/// Publisher for discrete states in body plan messages 未插值的离散状态规划消息发布者
	ros::Publisher discrete_body_plan_pub_;

	/// 在规划过程中遍历到的所有的状态消息发布者
	ros::Publisher all_state_pub_;

	/// Nodehandle to pub to and sub from     ros句柄
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data; 收发数据更新频率
	double update_rate_;

	/// 是否保存自定义数据结构FastTerrainMap的值到本地，保存后会终止程序，防止多次重复保存
	bool save_data_;

	/// 是否发布在规划过程中遍历到的所有的状态
	bool publish_all_state_;

	/// Number of times to call the planner   调用规划器的次数
	int num_calls_;

	/// Algorithm for planner to run (rrt-connect or rrt-star-connect)  运行的规划器算法
	std::string algorithm_;

	/// Time after which replanning is halted;  运行规划器的最短时间，当找到目标后，如果花费的时间超过该阈值，则不进行重新规划
	double replan_time_limit_;

	/// Handle for the map frame      map frame
	std::string map_frame_;

	/// Struct for terrain map data   高程图数据结构
	FastTerrainMap terrain_;

	/// Std vector containing the interpolated robot body plan  插值后机器人身体规划状态数组
	std::vector <State> body_plan_;

	/// Std vector containing the interpolated time data        插值后机器人身体规划时间数组
	std::vector<double> t_plan_;

	/// Robot starting state  机器人起点状态
	State robot_start_;

	/// Robot goal state      机器人终点状态
	State robot_goal_;

	/// Sequence of discrete states in the plan   规划中的离散状态序列
	std::vector <State> state_sequence_;

	/// Sequence of discrete actions in the plan  规划中的离散动作序列
	std::vector <Action> action_sequence_;

	/// Vector of cost instances in each planning call (nested STL vectors) 每次规划调用中的最优的路径质量数组
	std::vector <std::vector<double>> cost_vectors_;

	/// Vector of time instances of cost data for each planning call (nested STL vectors) 每次规划调用中找到最优路径质量所消耗的时间数组
	std::vector <std::vector<double>> cost_vectors_times_;

	/// Vector of solve times for each planning call  多次调用规划器分别花费的时间数组
	std::vector<double> solve_time_info_;

	/// Vector of number of vertices for each planning call 多次调用规划器分别树中生成的节点数
	std::vector<int> vertices_generated_info_;

	/// 在规划过程中遍历到的所有的状态
	std::vector<std::vector<double>> allStatePosition;

};


#endif // GLOBAL_BODY_PLANNER_H
