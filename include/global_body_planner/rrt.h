#ifndef RRT_H
#define RRT_H

#include "global_body_planner/planner_class.h"
#include <chrono>

#define TRAPPED 0	// 被困
#define ADVANCED 1
#define REACHED 2	// 到达

using namespace planning_utils;

//! A class that implements RRT sampling-based planning.	基于采样的RRT规划类
/*!
   This class builds an RRT using the PlannerClass as a data structure to maintain the tree. Methods to run the standard
   RRT algorithm are included, as well as utility methods to process and debug.
   这个类使用Planner类作为维护树的数据结构来构建一个RRT。 包括运行标准RRT算法的方法，以及用于处理和调试的实用方法。
*/
class RRTClass {
public:
	/**
	 * @brief Constructor for RRTClass	构造函数
	 * @return Constructed object of type RRTClass
	 */
	RRTClass();

	/**
	 * @brief Destructor for RRTClass	析构函数
	 */
	~RRTClass();

	/** Extend the tree towards the desired state	将树扩展到给定状态
	 * @param[in] T The PlannerClass instance containing the tree	包含树的PlannerClass对象
	 * @param[in] s The state to extend the tree towards	需要将树扩展到的状态
	 * @param[in] terrain Height map of the terrain			高程图
	 * @param[in] direction The direction with which to peform the extension (FORWARD to go away from the root vertex, REVERSE to go towards it)
	 * 						执行扩展的方向(FORWARD: 远离根节点，REVERSE: 靠近根节点)
	 */
	virtual int extend(PlannerClass &T, State s, FastTerrainMap &terrain, int direction);

	/**
	 * @brief Get the path from the root vertex to the specified one	返回根节点到指定节点的路径
	 * @param[in] T The PlannerClass instance containing the tree	包含树的 PlannerClass 实例
	 * @param[in] idx Index of the desired vertex	指定节点
	 * @return Vector of vertex indices on the path from the root of the tree to the desired
	 */
	std::vector<int> pathFromStart(PlannerClass &T, int idx);

	/**
	 * @brief Print the states in the specified path via stdout
	 * @param[in] T The PlannerClass instance containing the tree
	 * @param[in] path Vector of vertices in the path
	 */
	void printPath(PlannerClass &T, std::vector<int> path);

	/**
	 * @brief Execute the planner by constructing the tree until a path is found
	 * @param[in] terrain Height map of the terrain
	 * @param[in] s_start The start state of the planner
	 * @param[in] s_goal The goal state of the planner
	 * @param[out] state_sequence The sequence of states in the final path
	 * @param[out] action_sequence The sequence of actions in the final path
	 */
	void buildRRT(FastTerrainMap &terrain, State s_start, State s_goal, std::vector <State> &state_sequence,
				  std::vector <Action> &action_sequence);

	/**
	 * @brief Get the statistics for the planner solve	获取规划器求解的统计信息
	 * @param[out] plan_time The total time spent in the planner	在规划器中花费的总时间
	 * @param[out] success_var Number of solves under 5 seconds		5秒内解决的次数
	 * @param[out] vertices_generated Number of vertices generated in the tree	树中生成的节点数
	 * @param[out] time_to_first_solve The time elapsed until the first valid path was found	找到第一个有效路径之前经过的时间
	 * @param[out] length_vector 每次运行规划算法找到的最短路径长度
	 * @param[out] yaw_vector 每次运行规划算法找到的最小 yaw 旋转角度
	 * @param[out] cost_vector Vector of costs each time a better one is found	每次运行规划算法找到的最优的路径质量
	 * @param[out] cost_vector_times Vector of times at which each better cost is found	每次运行规划算法找到最优路径质量所消耗的时间
	 * @param[out] path_duration The duration of the path in seconds	路径的持续时间(单位：秒)
	 * @param[out] allStatePosition 树中探索到的所有的状态的三维空间位置
	 */
	void getStatistics(double &plan_time, int &success_var, int &vertices_generated, double &time_to_first_solve,
					   std::vector<double> &length_vector, std::vector<double> &yaw_vector,
					   std::vector<double> &cost_vector, std::vector<double> &cost_vector_times,
					   double &path_duration, std::vector<std::vector<double>> &allStatePosition);

	/**
	 * @brief Generate a new state that can be connected to the tree and is as close as possible to the specified state
	 *		  生成一个新的状态，可以连接到树上，并且尽可能接近指定的状态，并将其记录在s_new中，移动到s_new的动作记录在a_new中
	 * @param[in] s Specific state to move towards	要走向的指定状态
	 * @param[in] s_near Closest state in the tree to the specified state	树中最接近指定状态的状态
	 * @param[out] s_new New state yielded by taking an action from s_near	通过从 s_near 采取行动产生的新状态
	 * @param[out] a_new New action to take from s_near to get to s			从 s_near 到 s_new 的新动作
	 * @param[in] terrain Height map of the terrain							高程图
	 * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)	动态方向（FORWARD 或 REVERSE）
	 * @return Boolean if the new state got closer to the specified state than any other in the tree
	 */
	bool newConfig(State s, State s_near, State &s_new, Action &a_new, FastTerrainMap &terrain, int direction);

	/**
	 * @brief Get the states along the specified path of vertex indices	将路径中的节点index转换为状态
	 * @param[in] T The PlannerClass instance containing the tree		包含树的 PlannerClass 实例
	 * @param[in] path Vector of vertices in the path					指定路径
	 * @return The sequence of states in the path
	 */
	std::vector <State> getStateSequence(PlannerClass &T, std::vector<int> path);

	/**
	 * @brief Get the actions along the specified path of vertex indices (assumes that actions are synched with the states to which they lead)
	 * 		  沿着顶点索引的指定路径获取动作（假设动作与它们导致的状态同步）
	 * @param[in] T The PlannerClass instance containing the tree	包含树的 PlannerClass 实例
	 * @param[in] path Vector of vertices in the path				路径
	 * @return The sequence of actions in the path
	 */
	std::vector <Action> getActionSequence(PlannerClass &T, std::vector<int> path);

	/**
	 * @brief 将树中所有的状态记录到 allStatePosition 中，用于在rviz中显示
	 * @param[in] T The PlannerClass instance containing the tree		包含树的 PlannerClass 实例
	 */
	void saveStateSequence(PlannerClass &T);

	/**
	 * @brief 设置在动作采样时，根据相邻的两个状态速度变化方向，进行采样相关参数
	 */
	void set_action_direction_sampling(bool flag, double threshold);

	/**
	 * @brief 设置在状态采样时，根据起点或终点的位置，进行采样相关参数
	 */
	void set_state_direction_sampling(bool flag, double threshold, bool speed_direction_flag);

	/**
	 * @brief 设置自适应步长，当启用后，在对状态动作对进行有效性检测时，新状态如果是有效时，会增加检测时间分辨率步长
	 */
	void set_state_action_pair_check_adaptive_step_size_flag_(bool state_action_pair_check_adaptive_step_size_flag);

	/**
	 * @brief 路径质量中是否添加 yaw
	 */
	void set_cost_add_yaw(bool flag, double length_weight, double yaw_weight);

	/**
	 * @brief 打印优化输出参数
	 */
	void print_setting_parameters();

protected:

	/// Probability with which the goal is sampled (if running vanilla RRT)
	const double prob_goal_thresh = 0.05;

	/// Boolean for if the goal has been reached	是否到达目标
	bool goal_found = false;

	/// Total time elapsed in this planner call	此规划程序调用中经过的总时间
	std::chrono::duration<double> elapsed_total;

	// Total time elapsed until the first solve	第一次求解所用的总时间
	std::chrono::duration<double> elapsed_to_first;

	/// Integer checking if a solution was computed in 5 seconds	在5秒内计算出解决方案的次数
	int success_ = 0;

	/// Number of vertices in the tree	树中节点的数量
	int num_vertices;

	/// 路径长度(单位：米)
	double path_length_;

	/// 路径中 yaw 累计旋转角度
	double path_yaw_;

	/// 路径质量
	double path_cost_;

	/// 每次运行规划算法找到的最短路径长度
	std::vector<double> length_vector_;

	/// 每次运行规划算法找到的最小 yaw 旋转角度
	std::vector<double> yaw_vector_;

	/// Vector of costs each time a better one is found	每次运行规划算法找到的最优路径
	std::vector<double> cost_vector_;

	/// Vector of times at which each better cost is found	每次运行规划算法找到最优路径所消耗的时间
	std::vector<double> cost_vector_times_;

	/// The duration of the path in seconds	路径的持续时间(单位：秒)
	double path_duration_;

	/// 树中探索到的所有的状态的三维空间位置，用于在rviz中显示
	std::vector<std::vector<double>> allStatePosition_;

	/// 在动作采样时，根据相邻的两个状态速度变化方向，进行采样
	bool action_direction_sampling_flag_ = false;
	double action_direction_sampling_probability_threshold_ = 0.15;	// 概率阈值

	/// 在状态采样时，根据起点或终点的位置，进行采样
	bool state_direction_sampling_flag_ = false;
	double state_direction_sampling_probability_threshold_ = 0.05;	// 概率阈值
	bool state_direction_sampling_speed_direction_flag_ = false;	// 是否限制速度方向

	/// 自适应步长，当启用后，在对状态动作对进行有效性检测时，新状态如果是有效时，会增加检测时间分辨率步长
	bool state_action_pair_check_adaptive_step_size_flag_ = false;

	/// 路径质量中是否添加 yaw
	bool cost_add_yaw_flag_ = false;		// 是否启用
	double cost_add_yaw_length_weight_ = 1;	// 路径长度权重
	double cost_add_yaw_yaw_weight_ = 1;		// yaw 权重
};

#endif