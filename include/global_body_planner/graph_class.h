#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "global_body_planner/planning_utils.h"

using namespace planning_utils;

//! A general directed graph class.	一个一般的有向图类。
/*!
	This class implements a directed graph data structure with methods for adding and deleting vertices and edges,
	as well storing information at each vertex and providing print statements for debugging. Vertices are
	indexed with ints, and edges as unordered maps that map a vertex's index to the indices of its parents. If the
	graph is a tree there should only be one parent per vertex. Other unordered maps store information about each
	vertex, such as the associated state, action, or distance from the root vertex (g-value).
	该类实现了一个有向图的数据结构，具有添加和删除节点和边的方法。以及存储每个节点的信息，并提供用于调试的打印语句。
	节点是索引，而边则是无序的映射，它将一个节点的索引映射到其父辈的索引中。如果图是一棵树，每个节点应该只有一个父节点。
	其他的无序映射存储关于每个节点的信息，如相关的状态、动作或与根节点的距离（g-value）。
*/
class GraphClass {
public:
	/**
	 * @brief Constructor for GraphClass	构造函数
	 * @return Constructed object of type GraphClass
	 */
	GraphClass();

	/**
	 * @brief Destructor for GraphClass		析构函数
	 */
	~GraphClass();

	/**
	 * @brief Add a new vertex to the graph along with its state data	将一个新的节点连同其状态数据一起添加到图中
	 * @param[in] index Index of the new vertex	新节点的index
	 * @param[in] s State information corresponding to the specified index	与指定索引相对应的状态信息
	 */
	void addVertex(int index, State s);

	/**
	 * @brief Retrieve the state stored at a particular index in the graph	节点index -> 状态
	 * @param[in] index Index of the desired vertex
	 * @return State information corresponding to the requested index
	 */
	State getVertex(int index);

	/**
	 * @brief Retrieve the total number of vertices in the graph, computed as the size of the vertices vector	返回所有节点的数目
	 * @return Number of vertices in the graph
	 */
	int getNumVertices();

	/**
	 * @brief Add a new edge to the graph	向图中添加连接idx1与idx2的边
	 * @param[in] idx1 Index of the outgoing vertex of the edge
	 * @param[in] idx2 Index of the incoming vertex of the edge
	 */
	virtual void addEdge(int idx1, int idx2);

	/**
	 * @brief Remove an edge of the graph
	 * @param[in] idx1 Index of the outgoing vertex of the edge
	 * @param[in] idx2 Index of the incoming vertex of the edge
	 */
	void removeEdge(int idx1, int idx2);

	/**
	 * @brief Get the parent of a vertex	返回指定节点的父节点
	 * @param[in] idx Index of the desired vertex	指定节点
	 * @return Index of the parent of the specified vertex
	 */
	virtual int getPredecessor(int idx);

	/**
	 * @brief Get the children of a vertex
	 * @param[in] idx Index of the desired vertex
	 * @return Indices of the children of the specified vertex
	 */
	std::vector<int> getSuccessors(int idx);

	/**
	 * @brief Add an action to a particular vertex. This is the action that lead to this particular vertex from its parent.
	 * 		  向指定节点添加动作，该动作为从该节点的父节点走向该节点的动作
	 * @param[in] idx Index of the desired vertex	指定节点index
	 * @param[in] a Action corresponding to the desired vertex	指定动作
	 */
	void addAction(int idx, Action a);

	/**
	 * @brief Get the action of a vertex	返回指定节点的动作，该动作为从该节点的父节点走向该节点的动作
	 * @param[in] idx Index of the desired vertex	节点index
	 * @return Action corresponding to the desired vertex
	 */
	Action getAction(int idx);

	/**
	 * @brief Update the g-value of a vertex and propogate to all its successors	更新指定节点的g值，并更新他所有子节点的g值
	 * @param[in] idx Index of the desired vertex
	 * @param[in] val New g-value corresponding to the desired vertex
	 */
	void updateGValue(int idx, double val);

	/**
	 * @brief Get the g-value of a vertex	获取该节点到根节点的三维欧氏距离(g值)
	 * @param[in] idx Index of the desired vertex
	 * @return G-value corresponding to the desired vertex
	 */
	double getGValue(int idx);

	/**
	 * @brief Print the state information via stdout
	 * @param[in] s The state information to print
	 */
	void printVertex(State s);

	/**
	 * @brief Print the all vertices in the graph via stdout
	 */
	void printVertices();

	/**
	 * @brief Print the edges leading to a vertex via stdout
	 * @param[in] idx Index of the desired vertex
	 */
	void printIncomingEdges(int idx);

	/**
	 * @brief Print the all edges in the graph via stdout
	 */
	virtual void printEdges();

	/**
	 * @brief Initialize the graph by adding the root vertex (idx = 0) and setting g(idx) = 0
	 * 		  通过添加根节点(idx=0)并设置g(idx)=0来初始化图形。
	 * @param[in] s State for the root vertex
	 */
	virtual void init(State s);


protected:
	/// Map from vertex indices to corresponding states	节点index -> 相应状态
	std::unordered_map<int, State> vertices;

	/// Map from vertex indices to the actions leading to those vertices	节点index -> 从该节点的父节点指向该节点的动作
	std::unordered_map<int, Action> actions;

	/// Map from vertex indices to their parent(s)	节点index -> 该节点的父节点index
	std::unordered_map<int, std::vector<int> > edges;

	/// Map from vertex indices to their children	节点index -> 该节点的子节点index
	std::unordered_map<int, std::vector<int> > successors;

	/// Map from vertex indices to their costs (g-values)	节点index -> 该节点成本(g值，即与根节点的三维欧式距离)
	std::unordered_map<int, double> g_values;
};

#endif