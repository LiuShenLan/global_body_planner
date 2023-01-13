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
// s_from: 起点状态
// s_to: 终点状态
State PlannerClass::randomState(FastTerrainMap &terrain, bool state_direction_sampling_flag_, double state_direction_sampling_probability_threshold_, State s_from, State s_to) {
	State s;

	double probability = (double) rand() / RAND_MAX;	// 随机数概率

	if (state_direction_sampling_flag_ and probability <= state_direction_sampling_probability_threshold_)	// 启用
		s = randomStateDirection(terrain, s_from, s_to);
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
State PlannerClass::randomStateDirection(FastTerrainMap &terrain, State s_from, State s_to) {
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

// 通过多维欧氏距离获取距离指定状态最近的N个节点
std::vector<int> PlannerClass::neighborhoodN(State q, int N) {
	std::priority_queue <Distance, std::vector<Distance>, std::greater<Distance>> closest;	// 小根堆

	// 遍历所有节点，计算所有已有的节点与指定状态节点的距离，并使用优先队列排序
	std::unordered_map<int, State>::iterator itr;
	for (itr = vertices.begin(); itr != vertices.end(); itr++)
		closest.push(std::make_pair(stateDistance(q, itr->second), itr->first));

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
	for (itr = vertices.begin(); itr != vertices.end(); itr++) {
		if ((stateDistance(q, itr->second) <= dist) && stateDistance(q, itr->second) > 0) // don't include itself
		{
			neighbors.push_back(itr->first);
		}
	}

	return neighbors;
}

// 通过多维欧氏距离获取离指定状态最近的节点
int PlannerClass::getNearestNeighbor(State q) {
	std::vector<int> closest_q = neighborhoodN(q, 1);
	return closest_q.front();
}
