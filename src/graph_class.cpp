#include "global_body_planner/graph_class.h"

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <math.h>

using namespace planning_utils;

// 构造函数
GraphClass::GraphClass() {}

// 析构函数
GraphClass::~GraphClass() {}

// 节点index -> 状态
State GraphClass::getVertex(int idx) {
	return vertices[idx];
}

// 返回所有节点的数目
int GraphClass::getNumVertices() {
	return vertices.size();
}

// 将一个新的顶点连同其状态数据一起添加到图中
void GraphClass::addVertex(int idx, State q) {
	vertices[idx] = q;
	return;
}

// 向图中添加连接idx1与idx2的边
void GraphClass::addEdge(int idx1, int idx2) {
	edges[idx2].push_back(idx1);
	successors[idx1].push_back(idx2);
	g_values[idx2] = g_values[idx1] + poseDistance(vertices[idx1], vertices[idx2]);
	return;
}

void GraphClass::removeEdge(int idx1, int idx2) {
	std::vector<int>::iterator itr;
	for (itr = edges[idx2].begin(); itr != edges[idx2].end(); itr++) {
		if (*itr == idx1) {
			edges[idx2].erase(itr);
			break;
		}
	}
	for (itr = successors[idx1].begin(); itr != successors[idx1].end(); itr++) {
		if (*itr == idx2) {
			successors[idx1].erase(itr);
			break;
		}
	}
	return;
}

// 返回指定节点的父节点
int GraphClass::getPredecessor(int idx) {
	if (edges[idx].size() > 1) {
		std::cout << "More than one predecessor, fix this!" << std::endl;
		throw ("Error");
	}
	return edges[idx].front();
}

std::vector<int> GraphClass::getSuccessors(int idx) {
	return successors[idx];
}

// 向指定节点添加动作，该动作为从该节点的父节点走向该节点的动作
void GraphClass::addAction(int idx, Action a) {
	actions[idx] = a;
}

// 返回指定节点的动作，该动作为从该节点的父节点走向该节点的动作
Action GraphClass::getAction(int idx) {
	return actions[idx];
}

void GraphClass::printVertex(State vertex) {
	std::cout << "{";
	for (int i = 0; i < vertex.size(); i++)
		std::cout << vertex[i] << ", ";
	std::cout << "\b\b}";
}

void GraphClass::printVertices() {
	std::unordered_map<int, State>::iterator itr;
	std::cout << "\nAll Vertices : \n";
	for (itr = vertices.begin(); itr != vertices.end(); itr++) {
		std::cout << itr->first << " ";
		printVertex(itr->second);
		std::cout << std::endl;
	}
}

void GraphClass::printIncomingEdges(int in_vertex) {
	std::vector<int>::iterator itr;
	for (int i = 0; i < edges[in_vertex].size(); i++) {
		int out_vertex = edges[in_vertex][i];
		std::cout << "{" << out_vertex << " -> " << in_vertex << "} or ";
		printState(vertices[out_vertex]);
		std::cout << " -> ";
		printState(vertices[in_vertex]);
		std::cout << std::endl;
	}
}

void GraphClass::printEdges() {
	std::cout << "All Edges : \n";
	for (int i = 0; i < edges.size(); i++) {
		printIncomingEdges(i);
	}
}

// 获取该节点到根节点的三维欧氏距离
double GraphClass::getGValue(int idx) {
	return g_values[idx];
}

// 更新指定节点的g值，并更新他所有子节点的g值
void GraphClass::updateGValue(int idx, double val) {
	g_values[idx] = val;
	for (int successor: getSuccessors(idx)) {
		updateGValue(successor, g_values[idx] + poseDistance(getVertex(idx), getVertex(successor)));
	}
}

// 通过添加根顶点(idx=0)并设置g(idx)=0来初始化图形。
void GraphClass::init(State q) {
	int q_init = 0;
	addVertex(q_init, q);
	g_values[q_init] = 0;
	return;
}