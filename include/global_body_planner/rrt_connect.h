#ifndef RRTCONNECT_H
#define RRTCONNECT_H

#include "global_body_planner/rrt.h"
// #include "functions.h"

#define TRAPPED 0	// 被困
#define ADVANCED 1
#define REACHED 2	// 到达

using namespace planning_utils;

//! A class that implements RRT-Connect sampling-based planning.  基于采样的RRT-Connect规划类
/*!
   This class inherits the RRTClass, and adds members to connect two states exactly and post process the resulting solution.
   该类继承了RRTClass，并增加了成员来精确连接两个状态，并对结果解进行后处理
*/
class RRTConnectClass : public RRTClass {
public:
	/**
	 * @brief Constructor for RRTConnectClass 构造函数
	 * @return Constructed object of type RRTConnectClass
	 */
	RRTConnectClass();

	/**
	 * @brief Destructor for RRTConnectClass  析构函数
	 */
	~RRTConnectClass();

	/** Attempt to connect two states with specified stance time, and return a new state if the full connection is not possible
	 * 递归地尝试连接两个状态，如果无法完全连接则返回一个新状态，并将该状态存储到s_new中
	 * @param[in] s_existing The state that is already in the tree and closest to the specified state	已经在树中并且最接近指定状态的状态
	 * @param[in] s The state to extend the tree towards	需要将树延伸到的指定状态
	 * @param[in] t_s The stance time for this connection	此连接的支撑相时间
	 * @param[out] s_new New state yielded by taking the resulting action	输出：通过采取最终行动产生的新状态
	 * @param[out] a_new New action to connect the states	输出：连接各个状态的新行动
	 * @param[in] terrain Height map of the terrain		高程图
	 * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)	方向(FORWARD 或 REVERSE)
	 * @return Int describing the result of the attempt (TRAPPED, ADVANCED, or REACHED)	尝试的结果(TRAPPED, ADVANCED 或 REACHED)
	 */
	int attemptConnect(State s_existing, State s, double t_s, State &s_new, Action &a_new, FastTerrainMap &terrain,
					   int direction);

	/** Attempt to connect two states, and return a new state if the full connection is not possible. Internally computes stance time
	 * 尝试连接两个状态，如果无法完全连接则返回一个新状态，并将该状态存储到s_new中。 内部计算站立时间
	 * @param[in] s_existing The state that is already in the tree and closest to the specified state	已经在树中并且最接近指定状态的状态
	 * @param[in] s The state to extend the tree towards	需要将树延伸到的指定状态
	 * @param[out] s_new New state yielded by taking the resulting action	输出：通过采取最终行动产生的新状态
	 * @param[out] a_new New action to connect the states	输出：连接各个状态的新行动
	 * @param[in] terrain Height map of the terrain		高程图
	 * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)	方向(FORWARD 或 REVERSE)
	 * @return Int describing the result of the attempt (TRAPPED, ADVANCED, or REACHED)	尝试的结果(TRAPPED, ADVANCED 或 REACHED)
	 */
	int attemptConnect(State s_existing, State s, State &s_new, Action &a_new, FastTerrainMap &terrain, int direction);

	/** Connect the tree to the desired state	将树连接到指定状态
	 * @param[in] T The PlannerClass instance containing the tree	包含树的 PlannerClass 实例
	 * @param[in] s The state to connect the tree towards	需要连接的指定状态
	 * @param[in] terrain Height map of the terrain			高程图
	 * @param[in] direction The direction with which to peform the extension (FORWARD to go away from the root vertex, REVERSE to go towards it)
	 * 						执行扩展的方向(FORWARD: 远离根节点，REVERSE: 靠近根节点)
	 */
	int connect(PlannerClass &T, State s, FastTerrainMap &terrain, int direction);

	/**
	 * @brief Get the actions along the specified path of vertex indices (assumes that actions are synched with the state at which they are executed)
	 * 		  沿着指定路径的节点索引获取动作（假设动作与它们执行时的状态同步）
	 * @param[in] T The PlannerClass instance containing the tree	包含树的 PlannerClass 实例
	 * @param[in] path Vector of vertices in the path	路径
	 * @return The sequence of actions in the path
	 */
	std::vector<Action> getActionSequenceReverse(PlannerClass &T, std::vector<int> path);

	/**
	 * @brief Post process the path by removing extraneous states that can be bypassed	通过删除可以绕过的无关状态来对路径进行后处理优化
	 * @param[in] state_sequence The sequence of states in the path		状态序列
	 * @param[in] action_sequence The sequence of actions in the path	动作序列
	 * @param[in] terrain Height map of the terrain						高程图
	 */
	void postProcessPath(std::vector< State> &state_sequence, std::vector<Action> &action_sequence,
						 FastTerrainMap &terrain);

	/**
	 * @brief Run the RRT-Connect planner once until the goal is found	运行一次RRT-Connect计划器(起点树和终点树各尝试扩展到一个随机状态)，直到找到目标。
	 * @param[in] Ta Tree with root vertex at start state	根节点为初始状态的树
	 * @param[in] Tb Tree with root vertex at goal state	根节点为目标状态的树
	 * @param[in] terrain Height map of the terrain			高程图
	 */
	void runRRTConnect(PlannerClass &Ta, PlannerClass &Tb, FastTerrainMap &terrain);

	/**
	 * @brief Run the full RRT-Connect planner until the goal is found and time has expired, then post process and update statistics
	 * 		  运行完整的 RRT-Connect 规划器，直到找到目标并且时间已到，然后发布过程并更新统计信息
	 * @param[in] terrain Height map of the terrain			高程图
	 * @param[in] s_start The start state of the planner	规划器的初始状态
	 * @param[in] s_goal The goal state of the planner		规划器的目标状态
	 * @param[out] state_sequence The sequence of states in the final path		输出：最终路径中的状态序列
	 * @param[out] action_sequence The sequence of actions in the final path	输出：最终路径中的动作序列
	 * @param[in] max_time The time after which replanning is halted			停止重新规划的时间
	 */
	void buildRRTConnect(FastTerrainMap &terrain, State s_start, State s_goal, std::vector< State> &state_sequence,
						 std::vector<Action> &action_sequence, double max_time);

protected:

	/// Time horizon (in seconds) the planner is allowed to search until restarted	允许规划器搜索直到重新启动的时间范围(秒)
	double anytime_horizon;

	/// Initial guess at how quickly the planner executes (in m/s)	对规划器执行速度的初步猜测（单位：米/秒）
	const double planning_rate_estimate = 16.0; // m/s (meters planned/computation time)

	/// Initial anytime horizon (in seconds)	初始时间范围(单位：秒)
	double anytime_horizon_init;

	/// Factor by which horizon is increased if replanning is required	如果需要重新规划，视野增加因子(?)
	double horizon_expansion_factor = 1.2;

	/// Hard maximum time allowed for the planner, returns unsuccessfully if reached	规划为允许的最大时间，到达该时间阈值则返回失败	CHANGE: 20 -> 40
	const int max_time_solve = 600;
};

#endif