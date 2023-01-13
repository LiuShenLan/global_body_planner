#include "global_body_planner/rrt.h"
#include <ctime>

// constructor	构造函数
RRTClass::RRTClass() {}

// destructor	析构函数
RRTClass::~RRTClass() {}

using namespace planning_utils;

// 生成一个新的状态，可以连接到树上，并且尽可能接近指定的状态，并将其记录在s_new中，移动到s_new的动作记录在a_new中
// s: 要走向的指定状态
// s_near: 树中最接近指定状态的状态
// s_new: 通过从 s_near 采取行动产生的新状态
// a_new: 从 s_near 到 s_new 的新动作
// terrain: 高程图
// direction: 动态方向（FORWARD 或 REVERSE）
// 返回值：是否找到比树中所有状态都更接近指定状态的状态
bool RRTClass::newConfig(State s, State s_near, State &s_new, Action &a_new, FastTerrainMap &terrain, int direction) {
	double best_so_far = stateDistance(s_near, s);	// 计算要走向的状态与树中最接近要走向状态的状态之间的多维欧氏距离
	std::array<double, 3> surf_norm = terrain.getSurfaceNormal(s[0], s[1]);	// 高程图表面法线

	// std::array<double,3>
	for (int i = 0; i < NUM_GEN_STATES; ++i) {
		bool valid_state_found = false;	// 是否发现有效状态
		State s_test;	// 在走向指定状态中，遍历移动时间所能够走到的最远的有效状态
		double t_new;	// 似乎没用

		// 获取随机动作，其中使用表面法线对地面反作用力进行了旋转
		Action a_test = getRandomAction(surf_norm, direction, action_direction_sampling_flag_, action_direction_sampling_probability_threshold_, s, s_near);

		for (int j = 0; j < NUM_GEN_STATES; ++j) {
			bool is_valid;
			if (direction == FORWARD)	// 从起点往终点检查，前进，先支撑相后飞行相
				// s_near 走向 s
				is_valid = isValidStateActionPair(s_near, a_test, terrain, s_test, t_new, state_action_pair_check_adaptive_step_size_flag_);	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_test中
			else if (direction == REVERSE)	// 从终点往起点检查，后退，先飞行相后支撑相
				// s 走向 s_near
				is_valid = isValidStateActionPairReverse(s_near, a_test, terrain, s_test, t_new, state_action_pair_check_adaptive_step_size_flag_);	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中

			if (is_valid == true) {	// 发现有效状态
				valid_state_found = true;
				break;
			} else
				a_test = getRandomAction(surf_norm, direction, action_direction_sampling_flag_, action_direction_sampling_probability_threshold_, s, s_near);
		}

		if (valid_state_found == true) {	// 发现有效状态
			double current_dist = stateDistance(s_test, s);	// 从初始位置根据随机采样得到的动作得到的有效状态与要走向的指定状态之间的多维欧氏距离
			if (current_dist < best_so_far) {
				best_so_far = current_dist;
				s_new = s_test;
				a_new = a_test;
			}
		}
	}

	if (best_so_far == stateDistance(s_near, s))	// 最接近要走向的状态的状态就是树中状态
		return false;
	else
		return true;
}

// 将树扩展到给定状态
// T: 包含树的PlannerClass对象
// s: 需要将树扩展到的指定状态
// terrain: 高程图
// direction: 执行扩展的方向(FORWARD: 远离根节点，REVERSE: 靠近根节点)
int RRTClass::extend(PlannerClass &T, State s, FastTerrainMap &terrain, int direction) {
	int s_near_index = T.getNearestNeighbor(s);	// 通过欧氏距离获取离指定状态最近的节点的index
	State s_near = T.getVertex(s_near_index);	// 最近的状态节点
	State s_new;	// 尽可能接近给定状态s的输出新状态
	Action a_new;

	// 生成一个新的状态，可以连接到树上，并且尽可能接近指定的状态，并将其记录在s_new中
	if (newConfig(s, s_near, s_new, a_new, terrain, direction) == true) {

		// 向树中添加新状态
		int s_new_index = T.getNumVertices();	// 新状态index
		T.addVertex(s_new_index, s_new);		// 向图中添加新状态
		T.addEdge(s_near_index, s_new_index);	// 连接树中距离给定状态s最近的状态与新状态
		T.addAction(s_new_index, a_new);		// 连接两个状态
		T.updateGValue(s_new_index, T.getGValue(s_near_index) + poseDistance(s_near, s_new));	// 更新新状态到根节点的三维欧氏距离(g值)，并更新他所有子节点的g值

		// if (s_new == s)
		// 新状态与给定状态之间的多维欧式距离是否小于GOAL_BOUNDS(用于判断是否到达目标点)
		if (isWithinBounds(s_new, s) == true)
			return REACHED;
		else
			return ADVANCED;
	} else
		return TRAPPED;
}

// 返回根节点到指定节点的路径
// T: 包含树的 PlannerClass 实例
// idx: 指定节点
std::vector<int> RRTClass::pathFromStart(PlannerClass &T, int s) {
	std::vector<int> path;
	path.push_back(s);
	while (s != 0) {
		int s_pred = T.getPredecessor(s);	// s的父节点
		path.push_back(s_pred);
		s = s_pred;
	}
	std::reverse(path.begin(), path.end());
	return path;
}

// 将路径中的节点index转换为状态
// T: 包含树的 PlannerClass 实例
// path: 指定路径
std::vector<State> RRTClass::getStateSequence(PlannerClass &T, std::vector<int> path) {
	std::vector<State> state_sequence;
	for (int i = 0; i < path.size(); ++i)
		state_sequence.push_back(T.getVertex(path.at(i)));

	return state_sequence;
}

// 沿着指定路径的节点索引获取动作（假设动作与它们执行时的状态同步）
// T: 包含树的 PlannerClass 实例
// path: 路径
std::vector<Action> RRTClass::getActionSequence(PlannerClass &T, std::vector<int> path) {
	// Assumes that actions are synched with the states to which they lead
	// 假设动作与它们导致的状态同步	TODO: ?
	std::vector<Action> action_sequence;
	for (int i = 1; i < path.size(); ++i)
		action_sequence.push_back(T.getAction(path.at(i)));

	return action_sequence;
}

void RRTClass::printPath(PlannerClass &T, std::vector<int> path) {
	std::cout << "Printing path:";
	for (int i = 0; i < path.size(); i++) {
		std::cout << std::endl;
		std::cout << path.at(i) << " or ";
		printState(T.getVertex(path.at(i)));
		std::cout << " ->";
	}
	std::cout << "\b\b  " << std::endl;
}

void RRTClass::getStatistics(double &plan_time, int &success, int &vertices_generated, double &time_to_first_solve,
							 std::vector<double> &cost_vector, std::vector<double> &cost_vector_times,
							 double &path_duration, std::vector<std::vector<double>> &allStatePosition) {
	plan_time = elapsed_total.count();
	success = success_;
	vertices_generated = num_vertices;
	time_to_first_solve = elapsed_to_first.count();
	cost_vector = cost_vector_;
	cost_vector_times = cost_vector_times_;
	path_duration = path_duration_;
	allStatePosition = allStatePosition_;
	std::cout << "path_duration: " << path_duration << std::endl;
}

void RRTClass::buildRRT(FastTerrainMap &terrain, State s_start, State s_goal, std::vector<State> &state_sequence,
						std::vector<Action> &action_sequence) {
	auto t_start = std::chrono::high_resolution_clock::now();
	success_ = 0;

	cost_vector_.clear();
	cost_vector_times_.clear();

	std::cout << "RRT" << std::endl;

	PlannerClass T;
	T.init(s_start);

	int s_goal_idx;
	goal_found = false;
	while (true) {
		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;
		if (current_elapsed.count() >= 30) {
			num_vertices = T.getNumVertices();
			auto t_end = std::chrono::high_resolution_clock::now();
			elapsed_total = t_end - t_start;
			return;
		}

		// Generate new s to be either random or the goal
		double prob_goal = (double) rand() / RAND_MAX;
		State s = (prob_goal <= prob_goal_thresh) ? s_goal : T.randomState(terrain);

		if (isValidState(s, terrain, STANCE)) {
			int result = extend(T, s, terrain, FORWARD);

			if ((isWithinBounds(s, s_goal) == true) && (result == REACHED)) {
				std::cout << "goal found!" << std::endl;
				goal_found = true;
				s_goal_idx = T.getNumVertices() - 1;


				auto t_end = std::chrono::high_resolution_clock::now();
				elapsed_to_first = t_end - t_start;
				break;
			}
		}

	}

	num_vertices = T.getNumVertices();

	if (goal_found == true) {
		std::vector<int> path = pathFromStart(T, T.getNumVertices() - 1);
		state_sequence = getStateSequence(T, path);
		action_sequence = getActionSequence(T, path);
	} else {
		std::cout << "Path not found" << std::endl;
	}

	auto t_end = std::chrono::high_resolution_clock::now();
	elapsed_total = t_end - t_start;

	if (elapsed_total.count() <= 5.0)
		success_ = 1;

	path_duration_ = 0.0;
	for (Action a: action_sequence) {
		path_duration_ += a[6] + a[7];
	}

	path_quality_ = T.getGValue(s_goal_idx);
	cost_vector_.push_back(path_quality_);
	cost_vector_times_.push_back(elapsed_total.count());
	std::cout << "Path quality = " << path_quality_ << std::endl;
}

// 将树中所有的状态记录到 allStatePosition_ 中
// T: 包含树的 PlannerClass 实例
void RRTClass::saveStateSequence(PlannerClass &T) {
	int length = T.getNumVertices();	// 当前树中节点数
	State s;	// 树中状态
	std::vector<double> position(3, 0);	// 状态三维坐标
	// 遍历所有节点
	for (int i = 0; i < length; ++i) {
		s = T.getVertex(i);	// 节点 index -> 状态
		position[0] = s[0];
		position[1] = s[1];
		position[2] = s[2];
		allStatePosition_.push_back(position);
	}
}

// 设置在动作采样的时候，根据相邻的两个状态速度变化方向，进行采样相关参数
void RRTClass::set_action_direction_sampling_flag_(bool action_direction_sampling_flag) {
	action_direction_sampling_flag_ = action_direction_sampling_flag;
}
void RRTClass::set_action_direction_sampling_probability_threshold_(double action_direction_sampling_probability_threshold) {
	action_direction_sampling_probability_threshold_ = action_direction_sampling_probability_threshold;
}

// 设置在状态采样时，根据起点或终点的位置，进行采样相关参数
void RRTClass::set_state_direction_sampling_flag_(bool state_direction_sampling_flag) {
	state_direction_sampling_flag_ = state_direction_sampling_flag;
}
void RRTClass::set_state_direction_sampling_probability_threshold_(double state_direction_sampling_probability_threshold) {
	state_direction_sampling_probability_threshold_ = state_direction_sampling_probability_threshold;
}

// 设置自适应步长，当启用后，在对状态动作对进行有效性检测时，新状态如果是有效时，会增加检测时间分辨率步长
void RRTClass::set_state_action_pair_check_adaptive_step_size_flag_(bool state_action_pair_check_adaptive_step_size_flag) {
	state_action_pair_check_adaptive_step_size_flag_ = state_action_pair_check_adaptive_step_size_flag;
}
