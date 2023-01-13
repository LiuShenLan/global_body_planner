#include "global_body_planner/rrt_connect.h"

// constructor	构造函数
RRTConnectClass::RRTConnectClass() {}

// destructor	析构函数
RRTConnectClass::~RRTConnectClass() {}

using namespace planning_utils;

// 递归地尝试连接两个状态，如果无法完全连接则返回一个新状态，并将该状态存储到s_new中
// s_existing: 已经在树中并且最接近指定状态的状态
// s: 需要将树延伸到的指定状态
// t_s: 此连接的支撑相时间
// s_new: 通过采取最终行动产生的新状态
// a_new: 连接各个状态的新行动
// terrain: 高程图
// direction: 方向(FORWARD 或 REVERSE)
// 返回值: 尝试的结果(TRAPPED, ADVANCED 或 REACHED)
int RRTConnectClass::attemptConnect(State s_existing, State s, double t_s, State &s_new, Action &a_new,
									FastTerrainMap &terrain, int direction) {
	// 递归终止条件，若支撑相时间小于运动学检查时间分辨率，则该连接无效
	if (t_s <= KINEMATICS_RES)
		return TRAPPED;

	// Initialize the start and goal states depending on the direction, as well as the stance and flight times
	// 根据方向、支撑相和飞行相时间初始化初始和目标状态
	State s_start = (direction == FORWARD) ? s_existing : s;
	State s_goal = (direction == FORWARD) ? s : s_existing;
	double t_new;
	double t_f = 0;

	// Calculate the action to connect the desired states	计算连接所需状态的动作
	double x_td = s_start[0];
	double y_td = s_start[1];
	double z_td = s_start[2];
	double dx_td = s_start[3];
	double dy_td = s_start[4];
	double dz_td = s_start[5];

	double x_to = s_goal[0];
	double y_to = s_goal[1];
	double z_to = s_goal[2];
	double dx_to = s_goal[3];
	double dy_to = s_goal[4];
	double dz_to = s_goal[5];

	double p_td = s_start[6];
	double dp_td = s_start[7];
	double p_to = s_goal[6];
	double dp_to = s_goal[7];

	a_new[0] = -(2.0 * (3.0 * x_td - 3.0 * x_to + 2.0 * dx_td * t_s + dx_to * t_s)) / (t_s * t_s);
	a_new[1] = -(2.0 * (3.0 * y_td - 3.0 * y_to + 2.0 * dy_td * t_s + dy_to * t_s)) / (t_s * t_s);
	a_new[2] = -(2.0 * (3.0 * z_td - 3.0 * z_to + 2.0 * dz_td * t_s + dz_to * t_s)) / (t_s * t_s);
	a_new[3] = (2.0 * (3.0 * x_td - 3.0 * x_to + dx_td * t_s + 2.0 * dx_to * t_s)) / (t_s * t_s);
	a_new[4] = (2.0 * (3.0 * y_td - 3.0 * y_to + dy_td * t_s + 2.0 * dy_to * t_s)) / (t_s * t_s);
	a_new[5] = (2.0 * (3.0 * z_td - 3.0 * z_to + dz_td * t_s + 2.0 * dz_to * t_s)) / (t_s * t_s);
	a_new[6] = t_s;
	a_new[7] = t_f;

	a_new[8] = -(2.0 * (3.0 * p_td - 3.0 * p_to + 2.0 * dp_td * t_s + dp_to * t_s)) / (t_s * t_s);
	a_new[9] = (2.0 * (3.0 * p_td - 3.0 * p_to + dp_td * t_s + 2.0 * dp_to * t_s)) / (t_s * t_s);

	// If the connection results in an infeasible action, abort and return trapped	如果连接导致不可行的操作，则中止并返回被困
	if (isValidAction(a_new) == true) {	// 该动作有效(地面反作用力与摩擦锥符合条件)
		// 检查给定的状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
		bool isValid = (direction == FORWARD) ? (isValidStateActionPair(s_start, a_new, terrain, s_new, t_new, state_action_pair_check_adaptive_step_size_flag_))
											  : (isValidStateActionPairReverse(s_goal, a_new, terrain, s_new, t_new, state_action_pair_check_adaptive_step_size_flag_));

		// If valid, great, return REACHED, otherwise try again to the valid state returned by isValidStateActionPair
		if (isValid == true)	// 动作状态对有效，返回 REACHED
			return REACHED;
		else {	// 动作状态对无效，使用 isValidStateActionPair() 函数返回的新状态 s_new 作为待连接的状态，同时将结果存储到 s_new 中，注意引用与拷贝
			if (attemptConnect(s_existing, s_new, t_new, s_new, a_new, terrain, direction) == TRAPPED)
				return TRAPPED;
			else
				return ADVANCED;
		}
	}
	return TRAPPED;
}

// 尝试连接两个状态，如果无法完全连接则返回一个新状态，并将该状态存储到s_new中。 内部计算站立时间
// s_existing: 已经在树中并且最接近指定状态的状态
// s: 需要将树延伸到的指定状态
// s_new: 通过采取最终行动产生的新状态
// a_new: 连接各个状态的新行动
// terrain: 高程图
// direction: 方向(FORWARD 或 REVERSE)
// 返回值: 尝试的结果(TRAPPED, ADVANCED 或 REACHED)
int RRTConnectClass::attemptConnect(State s_existing, State s, State &s_new, Action &a_new, FastTerrainMap &terrain,
									int direction) {
	// select desired stance time to enforce a nominal stance velocity
	// 选择所需的支撑相时间以强制执行标准支撑相速度
	double t_s = poseDistance(s, s_existing) / V_NOM;
	return attemptConnect(s_existing, s, t_s, s_new, a_new, terrain, direction);
}


// 将树连接到指定状态
// T: 包含树的 PlannerClass 实例
// s: 需要连接的指定状态
// terrain: 高程图
// direction: 执行扩展的方向(FORWARD: 远离根节点，REVERSE: 靠近根节点)
int RRTConnectClass::connect(PlannerClass &T, State s, FastTerrainMap &terrain, int direction) {
	// Find nearest neighbor
	// 通过多维欧氏距离获取离指定状态最近的节点
	int s_near_index = T.getNearestNeighbor(s);
	State s_near = T.getVertex(s_near_index);
	State s_new;
	Action a_new;

	// 尝试连接到最近的邻居，如果结果为 REACHED 或 ADVANCED 则添加到图
	int result = attemptConnect(s_near, s, s_new, a_new, terrain, direction);
	if (result != TRAPPED) {
		int s_new_index = T.getNumVertices();	// 新状态节点index
		T.addVertex(s_new_index, s_new);		// 添加新状态节点
		T.addEdge(s_near_index, s_new_index);	// 连接新状态节点
		T.addAction(s_new_index, a_new);		// 添加从 s_near_index 走向 s_new_index 的动作 a_new
		T.updateGValue(s_new_index, T.getGValue(s_near_index) + poseDistance(s_near, s_new));	// 更新新节点的g值
	}

	return result;
}

// 沿着指定路径的节点索引获取动作（假设动作与它们执行时的状态同步）
// T: 包含树的 PlannerClass 实例
// path: 路径
std::vector<Action> RRTConnectClass::getActionSequenceReverse(PlannerClass &T, std::vector<int> path) {
	// Assumes that actions are synched with the states at which they are executed (opposite of the definition in RRTClass)
	// 假设动作与它们执行时的状态同步(与 RRTClass 中的定义相反)	TODO: ?
	std::vector<Action> action_sequence;
	for (int i = 0; i < path.size() - 1; ++i) {
		action_sequence.push_back(T.getAction(path.at(i)));
	}
	return action_sequence;
}

// 通过删除可以绕过的无关状态来对路径进行后处理优化
// state_sequence: 状态序列
// action_sequence: 动作序列
// terrain: 高程图
void RRTConnectClass::postProcessPath(std::vector<State> &state_sequence, std::vector<Action> &action_sequence,
									  FastTerrainMap &terrain) {
	auto t_start = std::chrono::high_resolution_clock::now();

	// Initialize first and last states
	State s = state_sequence.front();		// 起点状态
	State s_goal = state_sequence.back();	// 终点状态
	State s_next;
	State dummy;	// 临时存储变量，没有实际用处
	Action a_new;
	Action a_next;

	// Initialize state and action sequences
	std::vector<State> new_state_sequence;		// 优化后状态序列
	new_state_sequence.push_back(s);
	std::vector<Action> new_action_sequence;	// 优化后动作序列
	path_quality_ = 0;

	// 迭代直到目标被添加到状态序列
	while (s != s_goal) {
		std::vector<State> state_sequence_copy = state_sequence;	// 原始状态序列副本
		std::vector<Action> action_sequence_copy = action_sequence;	// 原始动作序列副本

		// 反向遍历状态序列得到s_next，尝试将当前状态s连接到s_next
		// 如果连接失败，则将s_next在状态序列中取前一个状态，直到连接成功或状态序列为空
		// 以获得从当前状态s开始获得一段最长的支撑相序列
		s_next = state_sequence_copy.back();	// 从序列的最后一个状态开始
		a_next = action_sequence_copy.back();
		State old_state;	// 连接失败时记录失败的状态，以便在遍历完整个状态序列都无法完成之后，将距离起点最近的下一个状态添加到状态序列之中
		Action old_action;
		while ((attemptConnect(s, s_next, dummy, a_new, terrain, FORWARD) != REACHED) && (s != s_next)) {
			// 连接失败，同时状态序列并不为空，可以将目标状态s_next取前一个更接近的状态进行连接
			old_state = s_next;
			old_action = a_next;

			// s_next取前一个状态
			state_sequence_copy.pop_back();
			action_sequence_copy.pop_back();
			s_next = state_sequence_copy.back();
			a_next = action_sequence_copy.back();
		}

		// If a new state was found add it to the sequence, otherwise add the next state in the original sequence
		if (s != s_next) {	// 连接成功，将成功连接的状态与动作添加到优化后的状态序列与动作序列，并记录路径质量
			new_state_sequence.push_back(s_next);
			new_action_sequence.push_back(a_new);
			path_quality_ += poseDistance(s, s_next);
			s = s_next;
		} else {	// 连接失败，则将起点的下一个状态直接添加到优化后的状态序列与动作序列
			new_state_sequence.push_back(old_state);
			new_action_sequence.push_back(old_action);
			path_quality_ += poseDistance(s, old_state);
			s = old_state;
		}
	}

	// 用优化后的状态序列和动作序列替换原始状态序列
	state_sequence = new_state_sequence;
	action_sequence = new_action_sequence;

	auto t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> processing_time = t_end - t_start;
}

// 运行一次RRT-Connect计划器(起点树和终点树各尝试扩展到一个随机状态)，直到找到目标。
void RRTConnectClass::runRRTConnect(PlannerClass &Ta, PlannerClass &Tb, FastTerrainMap &terrain) {
	auto t_start = std::chrono::high_resolution_clock::now();

	while (true) {
		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;

		// 超时后不再继续本次搜索
		if (current_elapsed.count() >= anytime_horizon) {	// 0.3 for plinth
			anytime_horizon = anytime_horizon * horizon_expansion_factor;
			// std::cout << "Took too long, retrying with horizon of " << anytime_horizon << "s" << std::endl;
			return;
		}

		// 从起点开始规划
		// State s_rand = Ta.randomState(terrain);	// Generate random s	通过在高程图范围内采样生成一个随机状态

		// 随机指向性状态采样
		State s_from = Ta.getVertex(Ta.getNumVertices() - 1), s_to = Tb.getVertex(0);
		State s_rand = Ta.randomState(terrain, state_direction_sampling_flag_, state_direction_sampling_probability_threshold_, s_from, s_to);

		// static int i = 0;	// TODO: 有用？

		if (isValidState(s_rand, terrain, STANCE)) {	// 检查随机状态是否有效

			// 尝试将树扩展到随机状态s_rand，并且没有被困住
			if (extend(Ta, s_rand, terrain, FORWARD) != TRAPPED) {
				State s_new = Ta.getVertex(Ta.getNumVertices() - 1);	// 最接近随机状态s_rand的状态

				// 尝试将新状态连接到终点树
				if (connect(Tb, s_new, terrain, REVERSE) == REACHED) {	// 连接成功，到达目标
					goal_found = true;

					auto t_end = std::chrono::high_resolution_clock::now();
					elapsed_to_first = t_end - t_start;
					path_quality_ = Ta.getGValue(Ta.getNumVertices() - 1) + Tb.getGValue(Tb.getNumVertices() - 1);	// 更新路径质量
					break;
				}
			}
		}

		// 从终点开始规划
		// s_rand = Tb.randomState(terrain);	// Generate random s	通过在高程图范围内采样生成一个随机状态

		// 随机指向性状态采样
		s_from = Ta.getVertex(0), s_to = Tb.getVertex(Tb.getNumVertices() - 1);
		s_rand = Tb.randomState(terrain, state_direction_sampling_flag_, state_direction_sampling_probability_threshold_, s_from, s_to);

		if (isValidState(s_rand, terrain, STANCE)) {	// 检查随机状态是否有效

			// 尝试将树扩展到随机状态s_rand，并且没有被困住
			if (extend(Tb, s_rand, terrain, REVERSE) != TRAPPED) {
				State s_new = Tb.getVertex(Tb.getNumVertices() - 1);	// 最接近随机状态s_rand的状态

				// 尝试将新状态连接到起点树
				if (connect(Ta, s_new, terrain, FORWARD) == REACHED) {	// 连接成功，到达目标
					goal_found = true;

					auto t_end = std::chrono::high_resolution_clock::now();
					elapsed_to_first = t_end - t_start;
					path_quality_ = Ta.getGValue(Ta.getNumVertices() - 1) + Tb.getGValue(Tb.getNumVertices() - 1);	// 更新路径质量
					break;
				}
			}
		}
	}
}

// 运行完整的 RRT-Connect 规划器，直到找到目标并且时间已到，然后发布过程并更新统计信息
// terrain: 高程图
// s_start: 规划器的初始状态
// s_goal: 规划器的目标状态
// state_sequence: 最终路径中的状态序列
// action_sequence: 最终路径中的动作序列
// max_time_opt: 停止重新规划的时间
void RRTConnectClass::buildRRTConnect(FastTerrainMap &terrain, State s_start, State s_goal,
									  std::vector<State> &state_sequence, std::vector<Action> &action_sequence,
									  double max_time_opt) {
	auto t_start = std::chrono::high_resolution_clock::now();
	success_ = 0;    // 在5秒内计算出解决方案的次数

	cost_vector_.clear();        // 每次找到更好的成本数组
	cost_vector_times_.clear();    // 每次找到更好的成本的时间数组

	std::cout << "RRT Connect" << std::endl;

	goal_found = false;    // 是否到达目标
	PlannerClass Ta;
	PlannerClass Tb;
	PlannerClass Ta_best;
	PlannerClass Tb_best;

	// Initialize anytime horizon	初始化允许规划器搜索直到重新启动的时间范围
	anytime_horizon = poseDistance(s_start, s_goal) / planning_rate_estimate;    // 允许规划器搜索直到重新启动的时间范围(秒)
	num_vertices = 0;    // 树中节点的数量

	double cost_so_far = INFTY;	// 最优路径质量(即最短路径长度)
	while (true) {
		Ta = PlannerClass();
		Tb = PlannerClass();
		Ta.init(s_start);    // 通过添加根节点(idx=0)并设置g(idx)=0来初始化图形。
		Tb.init(s_goal);

		goal_found = false;
		runRRTConnect(Ta, Tb, terrain);	// 运行一次RRT-Connect计划器(起点树和终点树各尝试扩展到一个随机状态)，直到找到目标。

		// 记录统计信息
		num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());	// 添加树中节点的数量
		// saveStateSequence(Ta);	// TODO: 记录树中所有状态
		// saveStateSequence(Tb);

		// 超时
		auto t_current = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> current_elapsed = t_current - t_start;	// 到此时规划所消耗的时间
		if (current_elapsed.count() >= max_time_solve) {	// 如果规划消耗时间大于设定阈值
			std::cout << "Failed, exiting" << std::endl;
			elapsed_total = current_elapsed;
			elapsed_to_first = current_elapsed;
			success_ = 0;
			num_vertices += (Ta.getNumVertices() + Tb.getNumVertices());
			return;
		}

		// 到达目标，路径后处理
		// 到达目标，将两棵树进行连接并计算得到状态序列与动作序列，之后通过删除可以绕过的无关状态来对路径进行后处理优化
		if (goal_found == true) {
			// Get both paths, remove the back of path_b and reverse it to align with path a
			// 计算根节点到最后向树中添加的节点的路径
			std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices() - 1);	// 节点index路劲	[起点, 中间状态]
			std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices() - 1);	// 节点index路径	[终点, 中间状态]

			// 反转路径b并删除路径b中与路径a重复的节点
			std::reverse(path_b.begin(), path_b.end());	// 反转以终点为根节点的路径	[中间状态, 终点]
			std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);	// 动作向量	[中间状态, 终点]
			path_b.erase(path_b.begin());	// 删除重复的中间状态节点	(中间状态, 终点]

			state_sequence = getStateSequence(Ta, path_a);	// 状态路径， [起点, 中间状态]
			std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);	// 状态路径	(中间状态, 终点]
			state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());	// 状态路径	[起点, 终点]

			action_sequence = getActionSequence(Ta, path_a);	// 动作向量	[起点, 中间状态]
			action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());	// 动作向量	[起点, 终点]

			postProcessPath(state_sequence, action_sequence, terrain);	// 通过删除可以绕过的无关状态来对路径进行后处理优化

			// 当前路径质量优于最优路径质量时，记录此时数据
			if (path_quality_ < cost_so_far) {
				cost_so_far = path_quality_;
				Ta_best = Ta;
				Tb_best = Tb;

				t_current = std::chrono::high_resolution_clock::now();
				current_elapsed = t_current - t_start;

				cost_vector_.push_back(cost_so_far);
				cost_vector_times_.push_back(current_elapsed.count());
			}
		}

		if ((goal_found == true) && (current_elapsed.count() >= max_time_opt))	// 找到目标并且超时，则不再继续进行规划
			break;
	}

	Ta = Ta_best;
	Tb = Tb_best;

	// 到达目标，将两棵树进行连接并计算得到状态序列与动作序列
	if (goal_found == true) {
		// Get both paths, remove the back of path_b and reverse it to align with path a
		std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices() - 1);
		std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices() - 1);

		std::reverse(path_b.begin(), path_b.end());
		std::vector<Action> action_sequence_b = getActionSequenceReverse(Tb, path_b);
		path_b.erase(path_b.begin());

		state_sequence = getStateSequence(Ta, path_a);
		std::vector<State> state_sequence_b = getStateSequence(Tb, path_b);
		state_sequence.insert(state_sequence.end(), state_sequence_b.begin(), state_sequence_b.end());

		action_sequence = getActionSequence(Ta, path_a);
		action_sequence.insert(action_sequence.end(), action_sequence_b.begin(), action_sequence_b.end());
	} else
		std::cout << "Path not found" << std::endl;

	postProcessPath(state_sequence, action_sequence, terrain);	// 通过删除可以绕过的无关状态来对路径进行后处理优化

	auto t_end = std::chrono::high_resolution_clock::now();
	elapsed_total = t_end - t_start;

	if (elapsed_total.count() <= 5.0)
		success_ = 1;

	// 记录路径持续的时间
	path_duration_ = 0.0;
	for (Action a: action_sequence)
		path_duration_ += (a[6] + a[7]);
}
