#include "global_body_planner/planner_class.h"
#include <queue>
#include <chrono>

// constructor	构造函数
PlannerClass::PlannerClass() {}

// destructor	析构函数
PlannerClass::~PlannerClass() {}

using namespace planning_utils;

typedef std::pair<double, int> Distance;

// 通过在高程图范围内采样生成一个随机状态，并决定是否采用指向性状态采样
// terrain: 高程图
// state_direction_sampling_flag: 是否启用指向性采样
// state_direction_sampling_probability_threshold: 启用后的概率阈值
// speed_direction_flag: 是否限制速度的方向
// s_from: 起点状态
// s_to: 终点状态
State PlannerClass::randomState(FastTerrainMap &terrain, bool state_direction_sampling_flag,
								double state_direction_sampling_probability_threshold, bool speed_direction_flag,
								State s_from, State s_to) {
	State s;

	double probability = (double) rand() / RAND_MAX;	// 随机数概率

	if (state_direction_sampling_flag and probability <= state_direction_sampling_probability_threshold)	// 启用
		s = randomStateDirection(terrain, s_from, s_to, speed_direction_flag);
	else	// 不启用
		s = randomState(terrain);

	return s;
}

// 通过在高程图范围内采样生成一个随机状态
State PlannerClass::randomState(FastTerrainMap &terrain) {
	// 高程图边界
	double x_min = terrain.getXData().front();
	double x_max = terrain.getXData().back();
	double y_min = terrain.getYData().front();
	double y_max = terrain.getYData().back();

	double z_min_rel = H_MIN + ROBOT_H;	// 底部最小离地间隙 + 机器人体高
	double z_max_rel = H_MAX + ROBOT_H;	// 底部最大高度 + 机器人体高

	State q;

	// Normal distribution sampling	正态分布采样
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();	// 随机数种子
	std::default_random_engine generator(seed);		// 随机数生成器
	// 高斯分布
	std::normal_distribution<double> height_distribution(
		0.5 * (z_max_rel + z_min_rel), (z_max_rel - z_min_rel) * (1.0 / (2 * 3.0))); // std is such that the max and min are 3 std away from mean
	std::normal_distribution<double> ang_vel_distribution(0.0, (P_MAX / 3.0)); // std is such that the max and min are 3 std away from mean

	q[0] = (x_max - x_min) * (double) rand() / RAND_MAX + x_min;	// x轴随机采样
	q[1] = (y_max - y_min) * (double) rand() / RAND_MAX + y_min;	// y轴随机采样
	q[2] = std::max(std::min(height_distribution(generator), z_max_rel), z_min_rel) +	// 高度高斯分布随机采样 + 地形高度值
		   terrain.getGroundHeight(q[0], q[1]);


	double phi = (2.0 * MY_PI) * (double) rand() / RAND_MAX;	// 速度与x轴正方向夹角，[0, 2 * pi]支架随机采样
	double cos_theta = 2.0 * (double) rand() / RAND_MAX - 1.0;
	double theta = acos(cos_theta);		// 速度与z轴正方向夹角，[0, pi]之间随机采样
	double v = (double) rand() / RAND_MAX * V_MAX;	// 速度，[0, V_MAX]之间随机采样
	q[3] = v * sin(theta) * cos(phi);	// dx
	q[4] = v * sin(theta) * sin(phi);	// dy
	q[5] = v * cos(theta);				// dz

	q[6] = 2 * P_MAX * (double) rand() / RAND_MAX - P_MAX;	// [-P_MAX, P_MAX]之间随机采样，即正负最大pitch采样
	q[7] = 0.0;

	return q;
}

// 通过在高程图范围内采样生成一个在 s_from 与 s_to 围成的矩形之中的状态
// terrain: 高程图
// s_from: 起点状态
// s_to: 终点状态
State PlannerClass::randomStateDirection(FastTerrainMap &terrain, State s_from, State s_to, bool speed_direction_flag) {
	// std::cout << "randomStateDirection()" << std::endl;
	// State: [0, 1, 2,  3,  4,  5, 6,  7]
	// index: [x, y, z, dx, dy, dz, p, dp]

	// 计算 s_from 与 s_to 的矩形边界
	double x_min = std::min(s_from[0], s_to[0]);
	double x_max = std::max(s_from[0], s_to[0]);
	double y_min = std::min(s_from[1], s_to[1]);
	double y_max = std::max(s_from[1], s_to[1]);

	// 高程图边界
	// double x_terrain_min = terrain.getXData().front();
	// double x_terrain_max = terrain.getXData().back();
	// double y_terrain_min = terrain.getYData().front();
	// double y_terrain_max = terrain.getYData().back();

	double z_min_rel = H_MIN + ROBOT_H;	// 底部最小离地间隙 + 机器人体高
	double z_max_rel = H_MAX + ROBOT_H;	// 底部最大高度 + 机器人体高

	State q;

	// Normal distribution sampling	正态分布采样
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();	// 随机数种子
	std::default_random_engine generator(seed);		// 随机数生成器
	// 高斯分布
	std::normal_distribution<double> height_distribution(
		0.5 * (z_max_rel + z_min_rel), (z_max_rel - z_min_rel) * (1.0 / (2 * 3.0))); // std is such that the max and min are 3 std away from mean
	std::normal_distribution<double> ang_vel_distribution(0.0, (P_MAX / 3.0)); // std is such that the max and min are 3 std away from mean

	q[0] = (x_max - x_min) * (double) rand() / RAND_MAX + x_min;	// x轴随机采样
	q[1] = (y_max - y_min) * (double) rand() / RAND_MAX + y_min;	// y轴随机采样
	q[2] = std::max(std::min(height_distribution(generator), z_max_rel), z_min_rel) +	// 高度高斯分布随机采样 + 地形高度值
		   terrain.getGroundHeight(q[0], q[1]);

	// 速度与x轴正方向夹角，[0, 2 * pi]支架随机采样
	double phi;
	if (speed_direction_flag) {	// 限制速度方向
		double delta_x = s_to[0] - s_from[0];
		double delta_y = s_to[1] - s_from[1];
		phi = std::atan2(delta_y, delta_x);
	} else
		phi = (2.0 * MY_PI) * (double) rand() / RAND_MAX;

	// TODO: 临时打印状态，记得删掉
	// std::cout << "from: ";
	// printStateXYZPYaw(s_from);
	// std::cout << "to  : ";
	// printStateXYZPYaw(s_to);
	// std::cout << "delta x: " << std::setw(7) << std::setprecision(3) << (s_to[0] - s_from[0]) << " | "
	// 		  << "y: " << std::setw(7) << std::setprecision(3) << (s_to[1] - s_from[1]) << " | "
	// 		  << "phi: " << phi << std::endl << std::endl;

	// 速度与z轴正方向夹角，[0, pi]之间随机采样
	double cos_theta = 2.0 * (double) rand() / RAND_MAX - 1.0;
	double theta = acos(cos_theta);
	// 速度，[0, V_MAX]之间随机采样
	double v = (double) rand() / RAND_MAX * V_MAX;
	q[3] = v * sin(theta) * cos(phi);	// dx
	q[4] = v * sin(theta) * sin(phi);	// dy
	q[5] = v * cos(theta);				// dz

	q[6] = 2 * P_MAX * (double) rand() / RAND_MAX - P_MAX;	// [-P_MAX, P_MAX]之间随机采样，即正负最大pitch采样
	q[7] = 0.0;

	return q;
}

// 通过多维欧氏距离获取距离指定状态最近的N个节点
std::vector<int> PlannerClass::neighborhoodN(State q, int N) {
	std::priority_queue <Distance, std::vector<Distance>, std::greater<Distance>> closest;	// 小根堆

	// 遍历所有节点，计算所有已有的节点与指定状态节点的距离，并使用优先队列排序
	std::unordered_map<int, State>::iterator itr;
	for (itr = vertices.begin(); itr != vertices.end(); itr++)
		closest.push(std::make_pair(stateDistance(q, itr->second, cost_add_yaw_flag_,
												  cost_add_yaw_length_weight_, cost_add_yaw_yaw_weight_),
									itr->first));

	if (N > closest.size())
		N = closest.size();

	std::vector<int> neighbors;
	for (int i = 0; i < N; i++) {
		neighbors.push_back(closest.top().second);
		closest.pop();
	}

	return neighbors;
}

std::vector<int> PlannerClass::neighborhoodDist(State q, double dist) {
	std::vector<int> neighbors;

	std::unordered_map<int, State>::iterator itr;
	for (itr = vertices.begin(); itr != vertices.end(); itr++)
		if ((stateDistance(q, itr->second) <= dist) && stateDistance(q, itr->second) > 0) // don't include itself
			neighbors.push_back(itr->first);

	return neighbors;
}

// 通过多维欧氏距离获取离指定状态最近的节点
int PlannerClass::getNearestNeighbor(State q) {
	int nearest_index = 0;	// 状态最近节点的index
	double cost_max = INFINITY;	// 最小距离
	double cost_temp;

	for (auto itr = vertices.begin(); itr != vertices.end(); ++itr) {
		// TODO: 扩展的时候是否要考虑 yaw
		// cost_temp = stateDistance(q, itr->second, cost_add_yaw_flag_, cost_add_yaw_length_weight_, cost_add_yaw_yaw_weight_);
		cost_temp = stateDistance(q, itr->second);
		if (cost_temp < cost_max) {
			cost_max = cost_temp;
			nearest_index = itr->first;
		}
	}
	return nearest_index;
}
