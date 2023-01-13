#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <limits>
#include <chrono>
#include <random>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <global_body_planner/fast_terrain_map.h>

namespace planning_utils {

	// Define kinematic constraint parameters	运动学约束参数
	const double H_MAX = 0.4;		// Maximum height of leg base, m									底部最大高度	原：0.6
	const double H_MIN = 0.075;		// Minimum ground clearance of body corners, m						底部最小离地间隙
	const double V_MAX = 2.0;		// Maximum robot velocity, m/s (4.0 for cheetah, 2.5 for anymal)	最大速度
	const double V_NOM = 0.75;		// Nominal velocity, m/s (used during connect function)				常规速度
	const double P_MAX = 1.0;		// Maximum pitch, rad												最大pitch
	const double DP_MAX = 3.0;		// Maximum angular velocity in pitch, rad/s							pitch最大角速度
	const double ANG_ACC_MAX = 7.0;	// Maximum angular acceleration in pitch, rad/s^2					pitch最大角加速度
	const double ROBOT_L = 0.3;		// Length of robot body, m (0.6 cheetah, 0.554 ANYmal)				机器人体长
	const double ROBOT_W = 0.3;		// Width of robot body, m (0.256 cheetah, 0.232 ANYmal)				机器人体宽
	const double ROBOT_H = 0.05;	// Vertical distance between leg base and bottom of robot, m (0.1 cheetah, 0.04 ANYmal)	机器人体高

	// Define dynamic constraint parameters	动态约束参数
	const double M_CONST = 13;		// Robot mass, kg (43 for cheetah, 30 for anymal)			机器人质量
	const double G_CONST = 9.81;	// Gravity constant, m/s^2				重力常数
	const double F_MAX = 637;		// Maximum GRF, N (800 for cheetah, 500 for anymal)			最大地面反作用力
	const double MU = 1.0;			// Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)	摩擦系数
	const double T_S_MIN = 0.3;		// Minimum stance time, s	最小支撑相时间
	const double T_S_MAX = 0.3;		// Maximum stance time, s	最大支撑相时间
	const double T_F_MIN = 0.0;		// Minimum flight time, s	最小飞行相时间
	const double T_F_MAX = 0.5;		// Maximum stance time, s	最大飞行相时间

	// Define planning parameters	规划参数
	const double KINEMATICS_RES = 0.05;	// Resolution of kinematic feasibility checks, s					运动学可行性检查时间分辨率
	const double BACKUP_TIME = 0.2;		// Duration of backup after finding an invalid state, s				发现无效状态后备份时长
	const double BACKUP_RATIO = 0.5;	// Ratio of trajectory to back up after finding an invalid state, s	RTT状态扩展时，发现无效状态后，将扩展出来的到达无效状态的轨迹回退的比例
	const int NUM_GEN_STATES = 6;		// Number of actions computed for each extend function				每个扩展函数所要计算的动作数
	const double GOAL_BOUNDS = 0.5;		// Distance threshold on reaching the goal (only used for vanilla RRT, not RRT-Connect)	到达目标的距离阈值

	// Define phase variable labels	相位变量标签
	const int FLIGHT = 0;			// 飞行相
	const int STANCE = 1;			// 支撑相
	const int CONNECT_STANCE = 2;	// 连接站立相，即该动作只有支撑相，没有飞行相，在支撑相运动之后是下一个动作的支撑相
	const int FORWARD = 0;			// RTT状态扩展时，离开根节点
	const int REVERSE = 1;			// RTT状态扩展时，朝向根节点

	// Define the dimensionality and types for states, actions, and pairs	状态、动作对的维度和类型
	const int POSEDIM = 3;		// 位姿维度
	const int STATEDIM = 8;		// 状态维度
	const int ACTIONDIM = 10;	// 动作维度
	typedef std::array<double, STATEDIM> State;		// 状态：x,y,z,dx,dy,dz,p,dp
	typedef std::array<double, ACTIONDIM> Action;	// 动作：支撑相开始时的加速度x, y, z, 支撑相结束时(飞行相)的加速度x, y, z, 支撑相时间, 飞行相时间, 支撑相开始时的俯仰加速度， 支撑相结束时的俯仰加速度
	typedef std::pair <State, Action> StateActionPair;

	// Define math parameters	数学参数
	const double INFTY = std::numeric_limits<double>::max();
	const double MY_PI = 3.14159;

	// Define some useful print statements
	void vectorToArray(State vec, double *new_array);
	void printState(State vec);
	void printVectorInt(std::vector<int> vec);
	void printStateNewline(State vec);
	void printVectorIntNewline(std::vector<int> vec);
	void printAction(Action a);
	void printActionNewline(Action a);
	void printStateSequence(std::vector <State> state_sequence);
	void printInterpStateSequence(std::vector <State> state_sequence, std::vector<double> interp_t);
	void printActionSequence(std::vector <Action> action_sequence);

	// Define some utility functions
	void interpStateActionPair(State s, Action a, double t0, double dt, std::vector <State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);	// 为状态-动作对插值
	void getInterpPath(std::vector <State> state_sequence, std::vector <Action> action_sequence, double dt, std::vector <State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);	// 根据动作序列为状态序列插值
	State interp(State q1, State q2, double x);
	double poseDistance(State q1, State q2);	// 计算两个状态对应位姿(x, y, z, q)之间的欧氏距离
	double stateDistance(State q1, State q2);	// 计算两个状态之间的多维欧氏距离
	bool isWithinBounds(State s1, State s2);	// 判断两个指定状态之间的多维欧式距离是否小于GOAL_BOUNDS(用于判断是否到达目标点)
	std::array<double, 3> rotate_grf(std::array<double, 3> surface_norm, std::array<double, 3> grf);	// 根据高程图表面法线，旋转地面反作用力，如果使用默认高程图表面法线，则不旋转

	// Define planning helper functions
	State applyStance(State s, Action a, double t);	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算下一个状态
	State applyStance(State s, Action a);	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算下一个状态，时间为支撑相时间
	State applyFlight(State s, double t_f);	// 在飞行相时，根据当前状态和飞行相时间，计算得到下一个状态，在飞行相中为匀速运动(垂直方向重力影响除外)
	State applyAction(State s, Action a);
	State applyStanceReverse(State s, Action a, double t);
	State applyStanceReverse(State s, Action a);
	Action getRandomAction(std::array<double, 3> surf_norm, int direction, bool action_direction_sampling_flag_, double action_direction_sampling_probability_threshold_, State s, State s_near);	// 获取随机动作，根据flag确定是否需要有指向性采样
	Action getRandomAction(std::array<double, 3> surf_norm);	// 获取随机动作
	Action getRandomActionDirection(std::array<double, 3> surf_norm, State s_from, State s_to);	// 获取有指向性的随机动作
	bool isValidAction(Action a);	// 检查一个动作是否是有效的(地面反作用力与摩擦锥)
	bool isValidState(State s, FastTerrainMap &terrain, int phase);

	// 检查状态动作对
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new, bool state_action_pair_check_adaptive_step_size_flag);	// 检查状态动作对，并判断是否使用自适应步长
	bool isValidStateActionPairAdaptiveStepSize(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new);	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中，添加自适应步长
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new);	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain);	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态

	// 检查反向动作状态对
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new, bool state_action_pair_check_adaptive_step_size_flag);	// 检查动作状态对，并判断是否使用自适应步长
	bool isValidStateActionPairReverseAdaptiveStepSize(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new);	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中，添加自适应步长
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new);	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain);	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
}

#endif