#ifndef PLANNERCLASS_H
#define PLANNERCLASS_H

#include "global_body_planner/graph_class.h"

using namespace planning_utils;

//! A directed graph class with supplemental methods to aid in sample-based planning.
// 一个带有补充方法的有向图类，以帮助进行基于采样的规划。

/*!
	This class inherits GraphClass, and adds method to add random states to the graph and search for neighbors.
	These functions are useful for sample-based planners such as RRTs or PRMs.
	该类继承了GraphClass，并增加了向图中添加随机状态和搜索邻居的方法。
	这些功能对于基于采样的规划器，如RRTs或PRMs，非常有用。
*/
class PlannerClass : public GraphClass {
public:
	/**
	 * @brief Constructor for PlannerClass	构造函数
	 * @return Constructed object of type PlannerClass
	 */
	PlannerClass();

	/**
	 * @brief Destructor for PlannerClass	析构函数
	 */
	~PlannerClass();

	/**
	 * @brief 通过在高程图范围内采样生成一个随机状态，并决定是否采用指向性状态采样
	 * @param[in] terrain 高程图
	 * @param[in] state_direction_sampling_flag 是否启用指向性采样
	 * @param[in] state_direction_sampling_probability_threshold 启用后的概率阈值
	 * @param[in] s_from 起点状态
	 * @param[in] s_to 终点状态
	 * @return Newly generated random state
	 */
	State randomState(FastTerrainMap &terrain, bool state_direction_sampling_flag_, double state_direction_sampling_probability_threshold_, State s_from, State s_to);

	/**
	 * @brief Generate a random state by sampling from within the bounds of the terrain	通过在高程图范围内采样生成一个随机状态
	 * @param[in] terrain Height map of the terrain	高程图
	 * @return Newly generated random state
	 */
	State randomState(FastTerrainMap &terrain);

	/**
	 * @brief 通过在高程图范围内采样生成一个状态，将 s_from 状态所处的位置按照平行于 xy 轴划分为四个区域，返回的新状态与终点状态在同一个象限
	 * @param[in] terrain 高程图
	 * @param[in] s_from 起点状态
	 * @param[in] s_to 终点状态
	 * @return Newly generated random state
	 */
	State randomStateDirection(FastTerrainMap &terrain, State s_from, State s_to);

	/**
	 * @brief Get the closest N vertices to the specified state by Euclidean distance.	通过多维欧氏距离获取距离指定状态最近的N个节点
	 * @param[in] s State to query the neighborhood						指定状态
	 * @param[in] N Number of vertices to include in the neighborhood	要包含在邻域中的节点数
	 * @return Vector of indices included in the neighborhood
	 */
	std::vector<int> neighborhoodN(State s, int N);

	/**
	 * @brief Get the vertices within the specified Euclidean distance of the specified state
	 * @param[in] s State to query the neighborhood
	 * @param[in] dist distance threshold for the neighborhood
	 * @return Vector of indices included in the neighborhood
	 */
	std::vector<int> neighborhoodDist(State q, double dist);

	/**
	 * @brief Get the closest verticex to the specified state by Euclidean distance.	通过多维欧氏距离获取离指定状态最近的节点
	 * @param[in] q State to query the neighborhood	指定状态
	 * @return Index of closest vertex
	 */
	int getNearestNeighbor(State q);

};

#endif