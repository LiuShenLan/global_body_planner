#include "global_body_planner/global_body_planner.h"

using namespace planning_utils;

// 构造函数
GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server 加载参数
	std::string terrain_map_topic, body_plan_topic, discrete_body_plan_topic;
	nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
	nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
	nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
	nh.param<std::string>("map_frame", map_frame_, "map");	// CHANGE: 由 /map 改为了 map
	nh.param<double>("global_body_planner/update_rate", update_rate_, 1);	// 收发数据更新频率
	nh.param<int>("global_body_planner/num_calls", num_calls_, 1);			// 调用规划器的次数
	nh.param<double>("global_body_planner/replan_time_limit", replan_time_limit_, 0.0);	// 运行规划器的最短时间，当找到目标后，如果花费的时间超过该阈值，则不进行重新规划
	nh.param<std::string>("global_body_planner/algorithm", algorithm_, "rrt-connect");	// 运行的规划器算法

	nh.param<bool>("global_body_planner/save_data", save_data_, false);	// 是否保存自定义数据结构FastTerrainMap的值到本地，保存后会终止程序，防止多次重复保存

	// Setup pubs and subs
	terrain_map_sub_ = nh_.subscribe(terrain_map_topic, 1, &GlobalBodyPlanner::terrainMapCallback, this);	// 高程图消息订阅者
	body_plan_pub_ = nh_.advertise<global_body_planner::BodyPlan>(body_plan_topic, 1);						// 插值后的规划消息发布者
	discrete_body_plan_pub_ = nh_.advertise<global_body_planner::BodyPlan>(discrete_body_plan_topic, 1);	// 身体规划中离散状态发布者

	// 是否发布在规划过程中遍历到的所有的状态
	nh.param<bool>("global_body_planner/publish_all_state", publish_all_state_, false);	// 是否发布在规划过程中遍历到的所有的状态
	if (publish_all_state_) {
		std::string all_state_topic;
		nh.param<std::string>("topics/all_state", all_state_topic, "/visualization/all_state");
		all_state_pub_ = nh_.advertise<visualization_msgs::Marker>(all_state_topic, 1);	// 在规划过程中遍历到的所有的状态消息发布者
	}
}

// 订阅到新的地图信息时更新地图数据
void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg) {
	// Get the map in its native form
	grid_map::GridMap map;
	grid_map::GridMapRosConverter::fromMessage(*msg, map);

	// Convert to FastTerrainMap structure for faster querying
	terrain_.loadDataFromGridMap(map);
}

// 清除规划变量
void GlobalBodyPlanner::clearPlan() {
	// Clear old solutions
	body_plan_.clear();
	t_plan_.clear();
	solve_time_info_.clear();
	vertices_generated_info_.clear();
	cost_vectors_.clear();
	cost_vectors_times_.clear();
}

// 调用正确的规划类并计算统计信息
void GlobalBodyPlanner::callPlanner() {

	// Get the most recent plan parameters and clear the old solutions
	setStartAndGoalStates();	// 设置规划器的起止点
	clearPlan();	// 清除规划变量

	// Initialize statistics variables	初始化统计变量
	double plan_time;			// 在规划器中花费的总时间
	int success;				// 5秒内解决的次数
	int vertices_generated;		// 树中生成的节点数
	double time_to_first_solve;	// 找到第一个有效路径之前经过的时间
	double path_duration;		// 路径的持续时间(单位：秒)
	double total_solve_time = 0;			// 在多次规划中消耗的总时间
	double total_vertices_generated = 0;	// 在多次规划中树中生成的节点数
	double total_path_length = 0;			// 在多次路径规划中，最优路径质量
	double total_path_duration = 0;			// 在多次路径规划中路径的持续时间(单位：秒)

	// Set up more objects
	cost_vectors_.reserve(num_calls_);
	cost_vectors_times_.reserve(num_calls_);
	RRTClass rrt_obj;		// 有用？
	RRTConnectClass rrt_connect_obj;
	RRTStarConnectClass rrt_star_connect_obj;

	// 设置规划器参数
	setPlannerParameter(rrt_obj);
	setPlannerParameter(rrt_connect_obj);
	setPlannerParameter(rrt_star_connect_obj);

	// Loop through num_calls_ planner calls 循环调用规划器num_calls_次
	for (int i = 0; i < num_calls_; ++i) {
		// 打印调用规划期次数
		std::cout << "----- plan times: " << (i + 1) << " / " << num_calls_ << " -----" << std::endl;	// 树中生成的节点数

		// Clear out previous solutions and initialize new statistics variables
		// 清除以前的解决方案并初始化统计变量
		state_sequence_.clear();	// 最终路径中的状态序列
		action_sequence_.clear();	// 最终路径中的动作序列
		std::vector<double> cost_vector;		// 每次运行规划算法找到的最优的路径质量
		std::vector<double> cost_vector_times;	// 每次运行规划算法找到最优路径质量所消耗的时间

		// Call the appropriate planning method (either RRT-Connect or RRT*-Connect)	调用适当的规划方法
		if (algorithm_.compare("rrt-connect") == 0) {
			// 运行完整的 RRT-Connect 规划器，直到找到目标并且时间已到，然后发布过程并更新统计信息
			rrt_connect_obj.buildRRTConnect(terrain_, robot_start_, robot_goal_, state_sequence_, action_sequence_,
											replan_time_limit_);
			// 获取规划器求解的统计信息
			rrt_connect_obj.getStatistics(plan_time, success, vertices_generated, time_to_first_solve, cost_vector,
										  cost_vector_times, path_duration, allStatePosition);
		} else if (algorithm_.compare("rrt-star-connect") == 0) {
			rrt_star_connect_obj.buildRRTStarConnect(terrain_, robot_start_, robot_goal_, state_sequence_,
													 action_sequence_, replan_time_limit_);
			rrt_star_connect_obj.getStatistics(plan_time, success, vertices_generated, time_to_first_solve, cost_vector,
											   cost_vector_times, path_duration, allStatePosition);
		} else
			throw std::runtime_error("Invalid algorithm specified");

		// Handle the statistical data	处理统计数据
		cost_vectors_.push_back(cost_vector);
		cost_vectors_times_.push_back(cost_vector_times);

		total_solve_time += plan_time;
		total_vertices_generated += vertices_generated;
		total_path_length += cost_vector.back();
		total_path_duration += path_duration;

		std::cout << "Vertices generated: " << vertices_generated << std::endl;		// 树中生成的节点数
		std::cout << "Solve time: " << plan_time << std::endl;				// 在规划器中花费的总时间
		std::cout << "Time to first solve: " << time_to_first_solve << std::endl;	// 找到第一个有效路径之前经过的时间
		std::cout << "Path length: " << cost_vector.back() << std::endl;	// 最优的路径质量

		solve_time_info_.push_back(plan_time);	// 多次调用规划器分别花费的时间数组
		vertices_generated_info_.push_back(vertices_generated);	// 多次调用规划器分别树中生成的节点数
	}

	// Report averaged statistics if num_calls_ > 1	如果多次调用规划器，则报告平均统计数据
	if (num_calls_ > 1) {
		std::cout << "---------- Average ----------" << std::endl;
		std::cout << "Average vertices generated: " << total_vertices_generated / num_calls_ << std::endl;
		std::cout << "Average solve time: " << total_solve_time / num_calls_ << std::endl;
		std::cout << "Average path length: " << total_path_length / num_calls_ << std::endl;
		std::cout << "Average path duration: " << total_path_duration / num_calls_ << std::endl;
	}

	// Interpolate to get full body plan	插值得到全部身体规划
	double dt = 0.05;	// 插值分辨率
	std::vector<int> interp_phase;	// 插值后机器人身体规划支撑相或飞行相数组
	getInterpPath(state_sequence_, action_sequence_, dt, body_plan_, t_plan_, interp_phase);	// 根据动作序列为状态序列插值
}

// 设置规划器的参数
void GlobalBodyPlanner::setPlannerParameter(RRTClass& rrt_obj) {
	// 在动作采样时，是否根据相邻的两个状态速度变化方向，进行采样
	bool action_direction_sampling_flag;
	double action_direction_sampling_probability_threshold;	// 概率阈值
	nh_.param<bool>("global_body_planner/action_direction_sampling/flag", action_direction_sampling_flag, false);
	nh_.param<double>("global_body_planner/action_direction_sampling/probability_threshold", action_direction_sampling_probability_threshold, 0.15);
	rrt_obj.set_action_direction_sampling_flag_(action_direction_sampling_flag);
	rrt_obj.set_action_direction_sampling_probability_threshold_(action_direction_sampling_probability_threshold);

	// 在状态采样时，根据起点或终点的位置，进行采样
	bool state_direction_sampling_flag;
	double state_direction_sampling_probability_threshold;	// 概率阈值
	nh_.param<bool>("global_body_planner/state_direction_sampling/flag", state_direction_sampling_flag, false);
	nh_.param<double>("global_body_planner/state_direction_sampling/probability_threshold", state_direction_sampling_probability_threshold, 0.15);
	rrt_obj.set_state_direction_sampling_flag_(state_direction_sampling_flag);
	rrt_obj.set_state_direction_sampling_probability_threshold_(state_direction_sampling_probability_threshold);

	// 是否使用自适应步长
	bool state_action_pair_check_adaptive_step_size_flag;
	nh_.param<bool>("global_body_planner/state_action_pair_check_adaptive_step_size_flag", state_action_pair_check_adaptive_step_size_flag, false);
	rrt_obj.set_state_action_pair_check_adaptive_step_size_flag_(state_action_pair_check_adaptive_step_size_flag);
}

// 设置规划器的起止点
void GlobalBodyPlanner::setStartAndGoalStates() {
	// Update any relevant planning parameters

	// x, y, z, dx, dy, dz, p, dp
	// yaw = atan2(dy, dx)
	// robot_start_ = {0,0,0.4,  0,0.1,0,  0,0};
	// robot_goal_ =  {8,0,0.4,  0,0,0,    0,0};

	// 读取起止点位置
	ros::NodeHandle nh = nh_;
	double start_x, start_y, start_z = 0.375, start_yaw;	// 起点
	nh.param<double>("state_publisher/start_position_x", start_x, 0);
	nh.param<double>("state_publisher/start_position_y", start_y, 0);
	// nh.param<double>("state_publisher/start_position_z", start_z, 0);
	nh.param<double>("state_publisher/start_yaw", start_yaw, 0);
	double goal_x, goal_y, goal_z = 0.375, goal_yaw;		//终点
	nh.param<double>("state_publisher/goal_position_x", goal_x, 0);
	nh.param<double>("state_publisher/goal_position_y", goal_y, 0);
	// nh.param<double>("state_publisher/goal_position_z", goal_z, 0);
	nh.param<double>("state_publisher/goal_yaw", goal_yaw, 0);

	// 计算起点的方向角 atan2(dy, dx)
	double delta = 0.0349066;	// 计算与 π / 2 之间的相等的冗余
	double start_dx = 1, start_dy = 0;
	if (start_yaw >= M_PI_2 - delta && start_yaw <= M_PI_2 + delta)			// start_yaw = π / 2
		start_dx = 0, start_dy = 1;
	else if (start_yaw >= -M_PI_2 - delta && start_yaw <= -M_PI_2 + delta)	// start_yaw = -π / 2
		start_dx = 0, start_dy = -1;
	else if (start_yaw > M_PI_2)	// start_yaw > π / 2
		start_dx = -1, start_dy = -std::tan(start_yaw - M_PI);
	else if (start_yaw < -M_PI_2)	// start_yaw < -π / 2
		start_dx = -1, start_dy = -std::tan(start_yaw + M_PI);
	else							// -π / 2 < start_yaw < π / 2
		start_dx = 1, start_dy = std::tan(start_yaw);
	// 计算终点的方向角 atan2(dy, dx)
	double goal_dx = 1, goal_dy = 0;
	if (goal_yaw >= M_PI_2 - delta && goal_yaw <= M_PI_2 + delta)			// goal_yaw = π / 2
		goal_dx = 0, goal_dy = 1;
	else if (goal_yaw >= -M_PI_2 - delta && goal_yaw <= -M_PI_2 + delta)	// goal_yaw = -π / 2
		goal_dx = 0, goal_dy = -1;
	else if (goal_yaw > M_PI_2)		// goal_yaw > π / 2
		goal_dx = -1, goal_dy = -std::tan(goal_yaw - M_PI);
	else if (goal_yaw < -M_PI_2)	// goal_yaw < -π / 2
		goal_dx = -1, goal_dy = -std::tan(goal_yaw + M_PI);
	else							// -π / 2 < goal_yaw < π / 2
		goal_dx = 1, goal_dy = std::tan(goal_yaw);

	robot_start_ = {start_x, start_y, start_z, start_dx, start_dy, 0, 0, 0};
	robot_goal_ = {goal_x, goal_y, goal_z, goal_dx, goal_dy, 0, 0, 0};
	ROS_INFO("start: x:%f, y:%f, z:%f, yaw:%f", robot_start_[0], robot_start_[1], robot_start_[2],
			 std::atan2(robot_start_[4], robot_start_[3]));
	ROS_INFO("goal: x:%f, y:%f, z:%f, yaw:%f", robot_goal_[0], robot_goal_[1], robot_goal_[2],
			 std::atan2(robot_goal_[4], robot_goal_[3]));

	robot_start_[2] += terrain_.getGroundHeight(robot_start_[0], robot_start_[1]);
	robot_goal_[2] += terrain_.getGroundHeight(robot_goal_[0], robot_goal_[1]);
}

// 将指定状态添加到规划信息中
void GlobalBodyPlanner::addBodyStateToMsg(double t, State body_state,
										  global_body_planner::BodyPlan &msg) {

	// Make sure the timestamps match the trajectory timing	确保时间戳与轨迹时间匹配
	ros::Duration time_elapsed(t);
	ros::Time current_time = msg.header.stamp + time_elapsed;

	// Represent each state as an Odometry message	将每个状态表示为里程计消息
	nav_msgs::Odometry state;
	state.header.frame_id = map_frame_;
	state.header.stamp = current_time;
	state.child_frame_id = "dummy";

	// Transform from RPY to quat msg
	tf2::Quaternion quat_tf;
	geometry_msgs::Quaternion quat_msg;
	quat_tf.setRPY(0, body_state[6], atan2(body_state[4], body_state[3]));
	quat_msg = tf2::toMsg(quat_tf);

	// 位置和方向
	state.pose.pose.position.x = body_state[0];	// x
	state.pose.pose.position.y = body_state[1];	// y
	state.pose.pose.position.z = body_state[2];	// z
	state.pose.pose.orientation = quat_msg;

	// 线速度与角速度
	state.twist.twist.linear.x = body_state[3];	// dx
	state.twist.twist.linear.y = body_state[4];	// dy
	state.twist.twist.linear.z = body_state[5];	// dz
	state.twist.twist.angular.x = 0;
	state.twist.twist.angular.y = body_state[6];
	state.twist.twist.angular.z = 0;

	msg.states.push_back(state);
}

// 发布在规划过程中遍历到的所有的状态
void GlobalBodyPlanner::publishAllState() {
	// Construct Marker message	构建标记信息
	visualization_msgs::Marker all_state_msg;

	// Initialize the headers and types	初始化header和类型
	ros::Time timestamp = ros::Time::now();
	all_state_msg.header.stamp = timestamp;
	all_state_msg.header.frame_id = map_frame_;
	all_state_msg.id = 0;
	all_state_msg.type = visualization_msgs::Marker::POINTS;

	// Define the shape of the discrete states	定义离散状态的形状
	double scale = 0.05;
	all_state_msg.scale.x = scale;
	all_state_msg.scale.y = scale;
	all_state_msg.scale.z = scale;
	all_state_msg.color.r = 0.733f;
	all_state_msg.color.a = 1.0;	// 透明度，越大越不透明

	// Loop through the discrete states	遍历离散状态
	int length = allStatePosition.size();
	for (int i = 0; i < length; i++) {
		geometry_msgs::Point p;
		p.x = allStatePosition[i][0];
		p.y = allStatePosition[i][1];
		p.z = allStatePosition[i][2];
		all_state_msg.points.push_back(p);
	}

	// 发布离散状态
	all_state_pub_.publish(all_state_msg);
}

// 将状态规划转换为ros消息并发布
void GlobalBodyPlanner::publishPlan() {
	// Construct BodyPlan messages	构造BodyPlan消息
	global_body_planner::BodyPlan body_plan_msg;
	global_body_planner::BodyPlan discrete_body_plan_msg;

	// Initialize the headers and types	初始化header和类型
	ros::Time timestamp = ros::Time::now();
	body_plan_msg.header.stamp = timestamp;
	body_plan_msg.header.frame_id = map_frame_;
	discrete_body_plan_msg.header = body_plan_msg.header;

	// 遍历插值后的规划，将状态添加到要发布的消息中
	for (int i = 0; i < body_plan_.size(); ++i)
		addBodyStateToMsg(t_plan_[i], body_plan_[i], body_plan_msg);

	// 遍历未插值的离散状态，并将状态添加到要发布的消息中
	for (int i = 0; i < state_sequence_.size(); i++)
		addBodyStateToMsg(t_plan_[i], state_sequence_[i], discrete_body_plan_msg);

	// 发布插值后的规划和未插值的离散状态规划
	body_plan_pub_.publish(body_plan_msg);
	discrete_body_plan_pub_.publish(discrete_body_plan_msg);

	// 发布在规划过程中遍历到的所有的状态
	if (publish_all_state_)
		publishAllState();
}

// 等待收到并处理地图消息
void GlobalBodyPlanner::waitForMap() {
	// Spin until terrain map message has been received and processed
	boost::shared_ptr < grid_map_msgs::GridMap const> shared_map;
	while ((shared_map == nullptr) && ros::ok()) {
		shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/terrain_map", nh_);
		ros::spinOnce();
	}

	if (save_data_)
		terrain_.saveData("/root/gbpl_ws/src/global_body_planner/data/temp");
}

// 类中的主要工作函数，在该组件的节点文件中调用
void GlobalBodyPlanner::spin() {
	ros::Rate r(update_rate_);

	waitForMap();	// 等待收到并处理地图消息

	callPlanner();	// Update the plan	调用正确的规划类并计算统计信息

	while (ros::ok()) {
		// If desired, get a new plan
		// updatePlan();

		// Publish the plan and sleep	发布规划并休眠
		publishPlan();	// 将状态规划转换为ros消息并发布
		ros::spinOnce();
		r.sleep();
	}
}