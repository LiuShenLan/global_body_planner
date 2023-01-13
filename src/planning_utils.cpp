#include "global_body_planner/planning_utils.h"

namespace planning_utils {
	// 将向量转为数组
	void vectorToArray(State vec, double *new_array) {
		for (int i = 0; i < vec.size(); i++)
			new_array[i] = vec.at(i);
	}

	// 打印状态
	void printState(State vec) {
		printf("{");
		for (int i = 0; i < vec.size(); i++)
			printf("%4.3f, ", vec[i]);
		printf("\b\b} ");
	}

	// 打印动作
	void printAction(Action a) {
		std::cout << "{";
		for (int i = 0; i < a.size(); i++)
			std::cout << a[i] << ", ";
		std::cout << "\b\b}";
	}

	// 打印int类型元素组成的向量
	void printVectorInt(std::vector<int> vec) {
		std::cout << "{";
		for (int i = 0; i < vec.size(); i++)
			std::cout << vec[i] << ", ";
		std::cout << "\b\b}";
	}

	// 打印xxx并附加新行(下同)
	void printStateNewline(State vec) {
		printState(vec);
		std::cout << std::endl;
	}

	void printActionNewline(Action a) {
		printAction(a);
		std::cout << std::endl;
	}

	// 打印状态序列
	void printStateSequence(std::vector <State> state_sequence) {
		for (State s: state_sequence)
			printStateNewline(s);
	}

	// 打印插值的状态序列(?)
	void printInterpStateSequence(std::vector <State> state_sequence, std::vector<double> interp_t) {
		for (int i = 0; i < state_sequence.size(); i++) {
			std::cout << interp_t[i] << "\t";
			printStateNewline(state_sequence[i]);
		}
	}

	// 打印动作序列
	void printActionSequence(std::vector <Action> action_sequence) {
		for (Action a: action_sequence)
			printActionNewline(a);
	}

	void printVectorInt_nl(std::vector<int> vec) {
		printVectorInt(vec);
		std::cout << std::endl;
	}

	// 在两个状态之间进行插值
	State interp(State q1, State q2, double x) {
		State q_out;
		for (int dim = 0; dim < q1.size(); dim++) {
			q_out[dim] = (q2[dim] - q1[dim]) * x + q1[dim];
		}
		return q_out;
	}

	// 计算两个状态对应位姿(x, y, z)之间的三维欧氏距离
	double poseDistance(State q1, State q2) {
		double sum = 0;
		for (int i = 0; i < POSEDIM; i++) {
			sum = sum + (q2[i] - q1[i]) * (q2[i] - q1[i]);
		}

		double dist = sqrt(sum);
		return dist;
	}

	// 计算两个状态之间的多维欧氏距离
	double stateDistance(State q1, State q2) {
		double sum = 0;
		State state_weight = {1, 1, 1, 1, 1, 1, 1, 1};

		for (int i = 0; i < q1.size(); i++) {
			sum = sum + state_weight[i] * (q2[i] - q1[i]) * (q2[i] - q1[i]);
		}

		double dist = sqrt(sum);
		return dist;
	}

	// 判断两个指定状态之间的多维欧式距离是否小于GOAL_BOUNDS(用于判断是否到达目标点)
	bool isWithinBounds(State s1, State s2) {
		return (stateDistance(s1, s2) <= GOAL_BOUNDS);
	}

	// 为状态-动作对插值
	// s: 给定状态
	// a: 给定动作
	// t0: 初始时间
	// dt: 动作时间
	// interp_path: 输出，插值后机器人身体规划状态数组
	// interp_t: 输出，插值后机器人身体规划时间数组
	// interp_phase: 输出，插值后机器人身体规划支撑相或飞行相数组
	void interpStateActionPair(State s, Action a, double t0, double dt, std::vector <State> &interp_path,
							   std::vector<double> &interp_t, std::vector<int> &interp_phase) {
		double t_s = a[6];	// 站立相时间
		double t_f = a[7];	// 飞行相时间

		// Add points during stance phase	在支撑相阶段插值
		for (double t = 0; t < t_s; t += dt) {
			interp_t.push_back(t + t0);
			interp_path.push_back(applyStance(s, a, t));
			if (t_f == 0)
				interp_phase.push_back(CONNECT_STANCE);
			else
				interp_phase.push_back(STANCE);
		}

		// Include the exact moment of liftoff	抬腿时刻的确切时间
		State s_takeoff = applyStance(s, a);

		// 飞行相插值
		for (double t = 0; t < t_f; t += dt) {
			interp_t.push_back(t_s + t + t0);
			interp_path.push_back(applyFlight(s_takeoff, t));
			interp_phase.push_back(FLIGHT);
		}

		// Include the exact landing state if there is a flight phase	如果有飞行相，则计算准确的落地状态
		if (t_f > 0) {
			interp_t.push_back(t0 + t_s + t_f);
			interp_path.push_back(applyFlight(s_takeoff, t_f));
			interp_phase.push_back(STANCE);
		}
	}

	// 根据动作序列为状态序列插值
	// state_sequence: 规划中的离散状态序列
	// action_sequence: 规划中的离散动作序列
	// dt: 插值分辨率
	// interp_path: 输出，插值后机器人身体规划状态数组
	// interp_t: 输出，插值后机器人身体规划时间数组
	// interp_phase: 输出，插值后机器人身体规划支撑相或飞行相数组
	void getInterpPath(std::vector <State> state_sequence, std::vector <Action> action_sequence, double dt,
					   std::vector <State> &interp_path, std::vector<double> &interp_t,
					   std::vector<int> &interp_phase) {
		double t0 = 0;	// 动作初始时间
		for (int i = 0; i < action_sequence.size(); i++) {	// 遍历离散动作序列
			interpStateActionPair(state_sequence[i], action_sequence[i], t0, dt, interp_path, interp_t, interp_phase);	// 为状态动作对插值
			t0 += (action_sequence[i][6] + action_sequence[i][7]);	// 修改初始时间
		}
		// 记录最后的状态与时间
		interp_t.push_back(t0);
		interp_path.push_back(state_sequence.back());
	}

	// 根据高程图表面法线，旋转地面反作用力，如果使用默认高程图表面法线，则不旋转
	// surface_norm: 高程图表面法线
	// grf: 地面反作用力
	std::array<double, 3> rotate_grf(std::array<double, 3> surface_norm, std::array<double, 3> grf) {
		// Receive data and convert to Eigen	接收数据并转换为 Eigen
		Eigen::Vector3d Zs;
		Zs << 0, 0, 1;

		Eigen::Vector3d surface_norm_eig;	// 高程图表面法线
		surface_norm_eig << surface_norm[0], surface_norm[1], surface_norm[2];

		// Normalize surface normal	归一化表面法线
		// surface_norm_eig.normalize();

		Eigen::Vector3d grf_eig;	// 地面反作用力
		grf_eig << grf[0], grf[1], grf[2];

		// Compute priors	计算先验
		Eigen::Vector3d v = surface_norm_eig.cross(Zs);	// 叉乘计算，平行向量叉乘结果是零向量
		double s = v.norm();
		double c = surface_norm_eig.dot(Zs);			// 点乘计算

		Eigen::Matrix3d vskew;
		vskew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;

		Eigen::Matrix3d R; // Rotation matrix to rotate from contact frame to spatial frame	从接frame旋转到空间frame的旋转矩阵
		double eps = 0.000001;
		if (s < eps)
			R = Eigen::Matrix3d::Identity();
		else
			R = Eigen::Matrix3d::Identity() + vskew + vskew * vskew * (1 - c) / (s * s);

		Eigen::Vector3d grf_spatial_eig = R * grf_eig;
		std::array<double, 3> grf_spatial = {grf_spatial_eig[0], grf_spatial_eig[1], grf_spatial_eig[2]};

		return grf_spatial;
	}

	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算下一个状态
	// s: 初始状态
	// a: 给定动作
	// t: 动作持续时间
	State applyStance(State s, Action a, double t) {
		double a_x_td = a[0];	// 支撑相加速度x
		double a_y_td = a[1];	// 支撑相加速度y
		double a_z_td = a[2];	// 支撑相加速度z
		double a_x_to = a[3];	// 飞行相加速度x
		double a_y_to = a[4];	// 飞行相加速度y
		double a_z_to = a[5];	// 飞行相加速度z
		double t_s = a[6];		// 支撑相时间

		double a_p_td = a[8];	// 支撑相俯仰加速度
		double a_p_to = a[9];	// 飞行相俯仰加速度

		double x_td = s[0];		// x
		double y_td = s[1];		// y
		double z_td = s[2];		// z
		double dx_td = s[3];	// dx
		double dy_td = s[4];	// dy
		double dz_td = s[5];	// dz

		double p_td = s[6];		// p
		double dp_td = s[7];	// dp

		State s_new;

		// 论文中公式(7)，从旧状态，对动作的速度、加速度积分得到新状态
		s_new[0] = x_td + dx_td * t + 0.5 * a_x_td * t * t + (a_x_to - a_x_td) * (t * t * t) / (6.0 * t_s);
		s_new[1] = y_td + dy_td * t + 0.5 * a_y_td * t * t + (a_y_to - a_y_td) * (t * t * t) / (6.0 * t_s);
		s_new[2] = z_td + dz_td * t + 0.5 * a_z_td * t * t + (a_z_to - a_z_td) * (t * t * t) / (6.0 * t_s);
		// 论文中公式(6)
		s_new[3] = dx_td + a_x_td * t + (a_x_to - a_x_td) * t * t / (2.0 * t_s);
		s_new[4] = dy_td + a_y_td * t + (a_y_to - a_y_td) * t * t / (2.0 * t_s);
		s_new[5] = dz_td + a_z_td * t + (a_z_to - a_z_td) * t * t / (2.0 * t_s);

		s_new[6] = p_td + dp_td * t + 0.5 * a_p_td * t * t + (a_p_to - a_p_td) * (t * t * t) / (6.0 * t_s);
		s_new[7] = dp_td + a_p_td * t + (a_p_to - a_p_td) * t * t / (2.0 * t_s);

		return s_new;
	}

	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算下一个状态，时间为支撑相时间
	// s: 初始状态
	// a: 给定动作
	State applyStance(State s, Action a) {
		return applyStance(s, a, a[6]);
	}

	// 在飞行相时，根据当前状态和飞行相时间，计算得到下一个状态，在飞行相中为匀速运动(垂直方向重力影响除外)
	// s: 初始状态
	// t_f: 飞行相时间
	State applyFlight(State s, double t_f) {
		double g = 9.81;	// 重力加速度
		double x_to = s[0];		// x
		double y_to = s[1];		// y
		double z_to = s[2];		// z
		double dx_to = s[3];	// dx
		double dy_to = s[4];	// dy
		double dz_to = s[5];	// dz

		double p_to = s[6];		// p
		double dp_to = s[7];	// dp

		State s_new;
		s_new[0] = x_to + dx_to * t_f;
		s_new[1] = y_to + dy_to * t_f;
		s_new[2] = z_to + dz_to * t_f - 0.5 * g * t_f * t_f;
		s_new[3] = dx_to;
		s_new[4] = dy_to;
		s_new[5] = dz_to - g * t_f;

		s_new[6] = p_to + dp_to * t_f;
		s_new[7] = dp_to;

		return s_new;
	}

	State applyAction(State s, Action a) {
		double t_s;
		double t_f;

		t_s = a[6];
		t_f = a[7];

		State s_to = applyStance(s, a);
		State s_new = applyFlight(s_to, t_f);
		return s_new;
	}

	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算前一个状态
	// s: 结束状态
	// a: 给定动作
	// t: 动作持续时间
	State applyStanceReverse(State s, Action a, double t) {
		double a_x_td = a[0];	// 支撑相加速度x
		double a_y_td = a[1];	// 支撑相加速度y
		double a_z_td = a[2];	// 支撑相加速度z
		double a_x_to = a[3];	// 飞行相加速度x
		double a_y_to = a[4];	// 飞行相加速度y
		double a_z_to = a[5];	// 飞行相加速度z
		double t_s = a[6];		// 支撑相时间

		double a_p_td = a[8];	// 支撑相俯仰加速度
		double a_p_to = a[9];	// 飞行相俯仰加速度

		double x_to = s[0];		// x
		double y_to = s[1];		// y
		double z_to = s[2];		// z
		double dx_to = s[3];	// dx
		double dy_to = s[4];	// dy
		double dz_to = s[5];	// dz

		double p_to = s[6];		// p
		double dp_to = s[7];	// dp

		State s_new;

		double cx = dx_to - a_x_td * t_s - 0.5 * (a_x_to - a_x_td) * t_s;
		double cy = dy_to - a_y_td * t_s - 0.5 * (a_y_to - a_y_td) * t_s;
		double cz = dz_to - a_z_td * t_s - 0.5 * (a_z_to - a_z_td) * t_s;

		double cp = dp_to - a_p_td * t_s - 0.5 * (a_p_to - a_p_td) * t_s;

		// // 论文中公式(7)
		s_new[0] = x_to - cx * (t_s - t) - 0.5 * a_x_td * (t_s * t_s - t * t) - (a_x_to - a_x_td) * (t_s * t_s * t_s - t * t * t) / (6.0 * t_s);
		s_new[1] = y_to - cy * (t_s - t) - 0.5 * a_y_td * (t_s * t_s - t * t) - (a_y_to - a_y_td) * (t_s * t_s * t_s - t * t * t) / (6.0 * t_s);
		s_new[2] = z_to - cz * (t_s - t) - 0.5 * a_z_td * (t_s * t_s - t * t) - (a_z_to - a_z_td) * (t_s * t_s * t_s - t * t * t) / (6.0 * t_s);
		// 论文中公式(6)
		s_new[3] = dx_to - a_x_td * (t_s - t) - (a_x_to - a_x_td) * (t_s * t_s - t * t) / (2.0 * t_s);
		s_new[4] = dy_to - a_y_td * (t_s - t) - (a_y_to - a_y_td) * (t_s * t_s - t * t) / (2.0 * t_s);
		s_new[5] = dz_to - a_z_td * (t_s - t) - (a_z_to - a_z_td) * (t_s * t_s - t * t) / (2.0 * t_s);

		s_new[7] = dp_to - a_p_td * (t_s - t) - (a_p_to - a_p_td) * (t_s * t_s - t * t) / (2.0 * t_s);
		s_new[6] = p_to - cp * (t_s - t) - 0.5 * a_p_td * (t_s * t_s - t * t) - (a_p_to - a_p_td) * (t_s * t_s * t_s - t * t * t) / (6.0 * t_s);

		return s_new;
	}

	// 在支撑相时，利用当前的状态和动作，根据论文中公式(6,7)计算前一个状态，时间为0，即整个支撑相与飞行相都在移动
	// s: 结束状态
	// a: 给定动作
	State applyStanceReverse(State s, Action a) {
		return applyStanceReverse(s, a, 0);
	}

	// 获取随机动作，根据 action_direction_sampling_flag_ 确定在动作采样的时候，是否根据相邻的两个状态速度变化方向，进行采样
	// surf_norm: 高程图表面法线
	// direction: 当启用时，状态之间的方向，FORWARD: 树中节点 s_near 走向 s；REVERSE: s 走向树中节点 s_near
	// action_direction_sampling_flag_: 是否启用flag
	// action_direction_sampling_probability_threshold_: 启用后的概率阈值
	// s: 树外节点
	// s_near: 树中节点
	Action getRandomAction(std::array<double, 3> surf_norm, int direction, bool action_direction_sampling_flag_, double action_direction_sampling_probability_threshold_, State s, State s_near) {
		Action a_test;
		double probability = (double) rand() / RAND_MAX;	// 随机数概率

		if (action_direction_sampling_flag_ and probability <= action_direction_sampling_probability_threshold_) {	// 启用
			if (direction == FORWARD)
				a_test = getRandomActionDirection(surf_norm, s_near, s);
			else
				a_test = getRandomActionDirection(surf_norm, s, s_near);
		} else	// 不启用，获取随机动作，其中使用表面法线对地面反作用力进行了旋转
			a_test = getRandomAction(surf_norm);
		return a_test;
	}

	// 获取随机动作
	// surf_norm: 高程图表面法线
	Action getRandomAction(std::array<double, 3> surf_norm) {
		Action a;

		// *_td: 支撑相开始时的*		*_to: 支撑相结束时的*

		// Random normal forces between 0 and F_MAX	在0和E_MAX之间随机采样得到触地和抬腿时的GRF(垂直)
		double f_z_td = F_MAX * (double) rand() / RAND_MAX;
		double f_z_to = F_MAX * (double) rand() / RAND_MAX;

		// Random tangential forces between -mu*f_z and mu*f_z	结合摩擦系数，计算正切方向的GRF，[-mu*f_z, mu*f_z]之间随机采样
		double f_x_td = 2 * MU * f_z_td * (double) rand() / RAND_MAX - MU * f_z_td;
		double f_x_to = 2 * MU * f_z_to * (double) rand() / RAND_MAX - MU * f_z_to;
		double f_y_td = 2 * MU * f_z_td * (double) rand() / RAND_MAX - MU * f_z_td;
		double f_y_to = 2 * MU * f_z_to * (double) rand() / RAND_MAX - MU * f_z_to;

		std::array<double, 3> f_td = {f_x_td, f_y_td, f_z_td};
		std::array<double, 3> f_to = {f_x_to, f_y_to, f_z_to};

		// 根据高程图表面法线，旋转地面反作用力，如果使用默认高程图表面法线，则不旋转
		f_td = rotate_grf(surf_norm, f_td);
		f_to = rotate_grf(surf_norm, f_to);

		// Random stance and flight times between 0 and T_MAX	在[0, T_MAX]之间随机采样得到支撑相时间和飞行相时间
		// double t_s = (T_S_MAX - T_S_MIN) * (double) rand() / RAND_MAX + T_S_MIN;	// 支撑相时间，[T_S_MIN, T_S_MAX]之间随机采样
		double t_s = 0.3;	// 支撑相时间
		double t_f = (T_F_MAX - T_F_MIN) * (double) rand() / RAND_MAX + T_F_MIN;	// 飞行相时间，[T_F_MIN, T_F_MAX]之间随机采样

		// TODO: 测试用，测完记得改回去
		// double t_s = (T_F_MAX - T_F_MIN) * (double) rand() / RAND_MAX + T_F_MIN;	// 支撑相时间，[T_F_MIN, T_F_MAX]之间随机采样
		// double t_f = (T_F_MAX - T_F_MIN) * (double) rand() / RAND_MAX + T_F_MIN;	// 飞行相时间，[T_F_MIN, T_F_MAX]之间随机采样

		// 计算得到动作
		a[0] = f_td[0] / M_CONST;			// 支撑相开始时的加速度x
		a[1] = f_td[1] / M_CONST;			// 支撑相开始时的加速度y
		a[2] = f_td[2] / M_CONST - G_CONST;	// 支撑相开始时的加速度z
		a[3] = f_to[0] / M_CONST;			// 支撑相结束时的加速度x
		a[4] = f_to[1] / M_CONST;			// 支撑相结束时的加速度y
		a[5] = f_to[2] / M_CONST - G_CONST;	// 支撑相结束时的加速度z
		a[6] = t_s;	// 支撑相时间
		a[7] = t_f;	// 飞行相时间

		// 对一个小方差的正态分布采样，获得角加速度(俯仰方向)
		// Randomly sample angular acceleration from a normal distribution with low std
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();	// 随机数种子
		std::default_random_engine generator(seed);		// 随机数生成器
		std::normal_distribution<double> ang_acc_distribution(0.0, (ANG_ACC_MAX / 4.0)); // std is such that the max and min are 4 std away from mean
		a[8] = std::max(std::min(ang_acc_distribution(generator), ANG_ACC_MAX), -ANG_ACC_MAX); //0.0; // for now
		a[9] = std::max(std::min(ang_acc_distribution(generator), ANG_ACC_MAX), -ANG_ACC_MAX); //0.0;

		return a;
	}

	// 获取有指向性的随机动作
	// surf_norm: 高程图表面法线
	// s_from: 起点状态
	// s_to: 终点状态
	Action getRandomActionDirection(std::array<double, 3> surf_norm, State s_from, State s_to) {
		// std::cout << "getRandomActionDirection()" << std::endl;	// TODO: 记得删除
		Action a;

		// State: [0, 1, 2,  3,  4,  5, 6,  7]
		// index: [x, y, z, dx, dy, dz, p, dp]

		// xyz方向的速度是否是增大的
		bool dx_increase_flag = s_to[3] > s_from[3];
		bool dy_increase_flag = s_to[4] > s_from[4];
		bool dz_increase_flag = s_to[5] > s_from[5];

		// *_td: 支撑相开始时的*		*_to: 支撑相结束时的*

		// 在0和E_MAX之间随机采样得到触地和抬腿时的GRF(垂直)
		double f_z_td = F_MAX * (double) rand() / RAND_MAX;
		double f_z_to = F_MAX * (double) rand() / RAND_MAX;
		// 摩擦力
		double f_friction_td = MU * f_z_td;
		double f_friction_to = MU * f_z_to;

		// 结合摩擦系数，计算正切方向的GRF
		// x 方向摩擦力
		double f_x_td, f_x_to;
		if (dx_increase_flag) {	// x 方向速度是增大的，[0, f_friction]
			f_x_td = f_friction_td * (double) rand() / RAND_MAX;
			f_x_to = f_friction_to * (double) rand() / RAND_MAX ;
		} else {	// x 方向速度是减小的，[-f_friction, 0]
			f_x_td = f_friction_td * (double) rand() / RAND_MAX - f_friction_td;
			f_x_to = f_friction_to * (double) rand() / RAND_MAX - f_friction_to;
		}
		// y 方向摩擦力
		double f_y_td, f_y_to;
		if (dy_increase_flag) {	// y 方向速度是增大的，[0, f_friction]
			f_y_td = f_friction_td * (double) rand() / RAND_MAX;
			f_y_to = f_friction_to * (double) rand() / RAND_MAX ;
		} else {	// y 方向速度是减小的，[-f_friction, 0]
			f_y_td = f_friction_td * (double) rand() / RAND_MAX - f_friction_td;
			f_y_to = f_friction_to * (double) rand() / RAND_MAX - f_friction_to;
		}

		std::array<double, 3> f_td = {f_x_td, f_y_td, f_z_td};
		std::array<double, 3> f_to = {f_x_to, f_y_to, f_z_to};

		// 根据高程图表面法线，旋转地面反作用力，如果使用默认高程图表面法线，则不旋转
		f_td = rotate_grf(surf_norm, f_td);
		f_to = rotate_grf(surf_norm, f_to);

		// Random stance and flight times between 0 and T_MAX	在[0, T_MAX]之间随机采样得到支撑相时间和飞行相时间
		// double t_s = (T_S_MAX - T_S_MIN) * (double) rand() / RAND_MAX + T_S_MIN;	// 支撑相时间，[T_S_MIN, T_S_MAX]之间随机采样
		double t_s = 0.3;	// 支撑相时间
		double t_f = (T_F_MAX - T_F_MIN) * (double) rand() / RAND_MAX + T_F_MIN;	// 飞行相时间，[T_F_MIN, T_F_MAX]之间随机采样

		// 计算得到动作
		a[0] = f_td[0] / M_CONST;			// 支撑相开始时的加速度x
		a[1] = f_td[1] / M_CONST;			// 支撑相开始时的加速度y
		a[2] = f_td[2] / M_CONST - G_CONST;	// 支撑相开始时的加速度z
		a[3] = f_to[0] / M_CONST;			// 支撑相结束时的加速度x
		a[4] = f_to[1] / M_CONST;			// 支撑相结束时的加速度y
		a[5] = f_to[2] / M_CONST - G_CONST;	// 支撑相结束时的加速度z
		a[6] = t_s;	// 支撑相时间
		a[7] = t_f;	// 飞行相时间

		// 对一个小方差的正态分布采样，获得角加速度(俯仰方向)
		// Randomly sample angular acceleration from a normal distribution with low std
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();	// 随机数种子
		std::default_random_engine generator(seed);		// 随机数生成器
		std::normal_distribution<double> ang_acc_distribution(0.0, (ANG_ACC_MAX / 4.0)); // std is such that the max and min are 4 std away from mean
		a[8] = std::max(std::min(ang_acc_distribution(generator), ANG_ACC_MAX), -ANG_ACC_MAX); //0.0; // for now
		a[9] = std::max(std::min(ang_acc_distribution(generator), ANG_ACC_MAX), -ANG_ACC_MAX); //0.0;

		return a;
	}

	// 检查一个动作是否是有效的(地面反作用力与摩擦锥)
	// a: 动作
	bool isValidAction(Action a) {

		if ((a[6] <= 0) || (a[7] < 0))	// 支撑相、飞行相时间
			return false;

		double m = M_CONST;	// 机器人质量
		double g = G_CONST;	// 重力常数
		double mu = MU;		// 摩擦系数

		// Get corresponding forces	计算相应的力
		double f_x_td = m * a[0];
		double f_y_td = m * a[1];
		double f_z_td = m * (a[2] + g);
		double f_x_to = m * a[3];
		double f_y_to = m * a[4];
		double f_z_to = m * (a[5] + g);

		double a_p_td = a[8];
		double a_p_to = a[9];

		// 检查总地面反作用力是否在允许的范围内(也即F_MAX以内)
		// Check force limits
		if ((sqrt(f_x_td * f_x_td + f_y_td * f_y_td + f_z_td * f_z_td) >= F_MAX) ||
			(sqrt(f_x_to * f_x_to + f_y_to * f_y_to + f_z_to * f_z_to) >= F_MAX) || (f_z_td < 0) || (f_z_to < 0) ||
			(a_p_td >= F_MAX) || (a_p_to >= F_MAX)) {
			// std::cout << "Force limits violated" << std::endl;
			return false;
		}

		// Check friction cone	检查摩擦锥的条件是否符合(保证机器人能够平稳)
		if ((sqrt(f_x_td * f_x_td + f_y_td * f_y_td) >= mu * f_z_td) ||
			(sqrt(f_x_to * f_x_to + f_y_to * f_y_to) >= mu * f_z_to)) {
			// std::cout << "Friction cone violated" << std::endl;
			return false;
		}

		return true;
	}

	// 检查状态是否有效
	// s: 状态
	// terrain: 高程图
	// phase: 支撑相 或 飞行相
	bool isValidState(State s, FastTerrainMap &terrain, int phase) {
		// 检查状态是否位于空缺值附近	TODO: 治标不治本
		if (terrain.heightIsNan(s[0], s[1]))
			return false;

		// 首先检查位姿是否在高程图范围内，以及俯仰角是否超过了最大值
		if ((s[0] < terrain.getXData().front()) || (s[0] > terrain.getXData().back()) ||	// x
			(s[1] < terrain.getYData().front()) || (s[1] > terrain.getYData().back()) ||	// y
			(abs(s[6]) >= P_MAX))	// pitch
			return false;

		// 检查速度是否超过最大值。忽略对垂直速度的限制，因为这是由重力精确限定的。
		if (sqrt(s[3] * s[3] + s[4] * s[4]) > V_MAX)
			return false;

		// Find yaw, pitch, and their sines and cosines	计算yaw、pitch及其相应的三角函数值
		double yaw = atan2(s[4], s[3]);
		double cy = cos(yaw);
		double sy = sin(yaw);
		double pitch = s[6];
		double cp = cos(pitch);
		double sp = sin(pitch);

		// Compute each element of the rotation matrix	计算旋转矩阵各元素的值
		double R_11 = cy * cp;
		double R_12 = -sy;
		double R_13 = cy * sp;
		double R_21 = sy * cp;
		double R_22 = cy;
		double R_23 = sy * sp;
		double R_31 = -sp;
		double R_32 = 0;
		double R_33 = cp;

		std::array<double, 2> test_x = {-0.5 * ROBOT_L, 0.5 * ROBOT_L};
		std::array<double, 2> test_y = {-0.5 * ROBOT_W, 0.5 * ROBOT_W};
		double z_body = -ROBOT_H;

		// Check each of the four corners of the robot	检查机器人的四个角
		for (double x_body: test_x) {
			for (double y_body: test_y) {
				// 腿的位置
				double x_leg = s[0] + R_11 * x_body + R_12 * y_body;
				double y_leg = s[1] + R_21 * x_body + R_22 * y_body;
				double z_leg = s[2] + R_31 * x_body + R_32 * y_body;	// 腿部最高点位置

				// 对应躯干底部角点的位置
				double x_corner = x_leg + R_13 * z_body;
				double y_corner = y_leg + R_23 * z_body;
				double z_corner = z_leg + R_33 * z_body;

				// 检查腿是否位于空缺值附近	TODO: 治标不治本
				if (terrain.heightIsNan(x_leg, y_leg))
					return false;

				// 计算腿部高度以及躯干底部角点的高度
				double leg_height = z_leg - terrain.getGroundHeight(x_leg, y_leg);	// 腿部最高点离地距离
				double corner_height = z_corner - terrain.getGroundHeight(x_corner, y_corner);
				// std::cout << "x,y,z = (" << x_spatial << ", " << y_spatial << ", " << z_spatial << "), height = " <<
				//     z_spatial << " - " << terrain.getGroundHeight(x_spatial,y_spatial) << " = " << height << std::endl;

				// Always check for collision, only check reachability in stance	检查躯干是否与地面碰撞，支撑相同时检查腿是否触地
				if ((corner_height < H_MIN) || ((phase == STANCE) && (leg_height > H_MAX)))
					return false;
			}
		}

		// Check the center of the underside of the robot	检查机器人底部中心距离地面的高度
		double height = (s[2] + R_33 * z_body) - terrain.getGroundHeight(s[0] + R_13 * z_body, s[1] + R_23 * z_body);
		if (height < H_MIN)
			return false;

		return true;
	}

	// 检查状态动作对，并判断是否使用自适应步长
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new, bool state_action_pair_check_adaptive_step_size_flag) {
		if (state_action_pair_check_adaptive_step_size_flag)
			return isValidStateActionPairAdaptiveStepSize(s, a, terrain, s_new, t_new);
		else
			return isValidStateActionPair(s, a, terrain, s_new, t_new);
	}

	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中，添加自适应步长
	// s: 树中状态节点，即为初始状态
	// a: 给定动作
	// terrain: 高程图
	// s_new: 在给定初始状态和动作的情况下，遍历移动时间所能够走到的最远的有效状态
	// t_new: 到达s_new的时间
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPairAdaptiveStepSize(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new) {
		// std::cout << "isValidStateActionPairAdaptiveStepSize()" << std::endl;
		double t_s = a[6];	// 支撑相时间
		double t_f = a[7];	// 飞行相时间

		// 自适应步长参数
		double time_step = KINEMATICS_RES;
		double t_pre_success = 0;

		// 遍历整个支撑相时间，检查支撑相阶段的轨迹是否均为有效状态，同时记录所能到达的最远有效状态到s_new
		for (double t = 0; t <= t_s; t += time_step) {
			// 根据初始状态与给定动作，计算出下一个状态
			State s_check = applyStance(s, a, t);

			if (isValidState(s_check, terrain, STANCE) == false) {	// 计算出的状态无效
				// 当前步长接近分辨率，即动作失败
				if (KINEMATICS_RES - 0.01 <= time_step and time_step <= KINEMATICS_RES + 0.01) {
					// 将计算出来的到达无效状态的轨迹回退指定比例后，记录此时到达的有效状态
					s_new = applyStance(s, a, (1.0 - BACKUP_RATIO) * t);
					return false;
				} else {	// 当前步长大于分辨率，可能因为步长过长而走到了障碍物处，减小步长
					time_step = KINEMATICS_RES;
					t = t_pre_success;
				}
			} else {	// 计算出的状态是有效的，记录此时到达的有效状态
				s_new = s_check;
				t_new = t;

				// 记录自适应步长参数
				time_step += KINEMATICS_RES;
				t_pre_success = t;
			}
		}

		State s_takeoff = applyStance(s, a);	// 抬腿时刻的状态
		time_step = KINEMATICS_RES;
		t_pre_success = 0;

		// 遍历飞行相时间，检查飞行相阶段的轨迹是否均为有效状态
		for (double t = 0; t < t_f; t += time_step) {
			State s_check = applyFlight(s_takeoff, t);	// 根据抬腿时刻的状态和飞行相时间，计算得到下一个状态

			// 若下一个状态无效，则抛弃掉整段飞行相轨迹
			if (isValidState(s_check, terrain, FLIGHT) == false) {
				if (KINEMATICS_RES - 0.01 <= time_step and time_step <= KINEMATICS_RES + 0.01)
					return false;
				else {
					time_step = KINEMATICS_RES;
					t = t_pre_success;
				}
			} else {	// 下一个状态有效，记录自适应步长参数
				time_step += KINEMATICS_RES;
				t_pre_success = t;
			}
		}

		State s_land = applyFlight(s_takeoff, t_f);	// 落地时的状态
		if (isValidState(s_land, terrain, STANCE) == false)	// 若落地时的状态无效，则直接抛弃整段飞行相轨迹
			return false;
		else {	// 落地状态有效，记录此时的落地状态
			s_new = s_land;
			t_new = t_s + t_f;
		}

		// 站立相和飞行相轨迹完全有效
		return true;
	}

	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
	// s: 树中状态节点，即为初始状态
	// a: 给定动作
	// terrain: 高程图
	// s_new: 在给定初始状态和动作的情况下，遍历移动时间所能够走到的最远的有效状态
	// t_new: 到达s_new的时间
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new) {
		double t_s = a[6];	// 支撑相时间
		double t_f = a[7];	// 飞行相时间

		// 遍历整个支撑相时间，检查支撑相阶段的轨迹是否均为有效状态，同时记录所能到达的最远有效状态到s_new
		for (double t = 0; t <= t_s; t += KINEMATICS_RES) {
			// 根据初始状态与给定动作，计算出下一个状态
			State s_check = applyStance(s, a, t);

			if (isValidState(s_check, terrain, STANCE) == false) {	// 计算出的状态无效
				s_new = applyStance(s, a, (1.0 - BACKUP_RATIO) * t);	// 将计算出来的到达无效状态的轨迹回退指定比例后，记录此时到达的有效状态
				// s_new = applyStance(s,a,(t - BACKUP_TIME));
				return false;
			} else {	// 计算出的状态是有效的，记录此时到达的有效状态
				s_new = s_check;
				t_new = t;
			}
		}

		State s_takeoff = applyStance(s, a);	// 抬腿时刻的状态

		// 遍历飞行相时间，检查飞行相阶段的轨迹是否均为有效状态
		for (double t = 0; t < t_f; t += KINEMATICS_RES) {
			State s_check = applyFlight(s_takeoff, t);	// 根据抬腿时刻的状态和飞行相时间，计算得到下一个状态

			// 若下一个状态无效，则抛弃掉整段飞行相轨迹
			if (isValidState(s_check, terrain, FLIGHT) == false)
				return false;
		}

		State s_land = applyFlight(s_takeoff, t_f);	// 落地时的状态
		if (isValidState(s_land, terrain, STANCE) == false)	// 若落地时的状态无效，则直接抛弃整段飞行相轨迹
			return false;
		else {	// 落地状态有效，记录此时的落地状态
			s_new = s_land;
			t_new = t_s + t_f;
		}

		// 站立相和飞行相轨迹完全有效
		return true;
	}

	// 给定初始状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态
	// s: 树中状态节点，即为初始状态
	// a: 给定动作
	// terrain: 高程图
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPair(State s, Action a, FastTerrainMap &terrain) {
		State dummy_state;
		double dummy_time;
		return isValidStateActionPair(s, a, terrain, dummy_state, dummy_time);
	}

	// 检查动作状态对，并判断是否使用自适应步长
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new, bool state_action_pair_check_adaptive_step_size_flag) {
		if (state_action_pair_check_adaptive_step_size_flag)
			return isValidStateActionPairReverseAdaptiveStepSize(s, a, terrain, s_new, t_new);
		else
			return isValidStateActionPairReverse(s, a, terrain, s_new, t_new);
	}

	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中，添加自适应步长
	// s: 树中状态节点，即为结束状态
	// a: 给定动作
	// terrain: 高程图
	// s_new: 在给定结束状态和动作的情况下，遍历移动时间所能够走到的最远的有效状态
	// t_new: 到达s_new的时间
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPairReverseAdaptiveStepSize(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new) {
		// std::cout << "isValidStateActionPairReverseAdaptiveStepSize()" << std::endl;
		double t_s = a[6];	// 支撑相时间
		double t_f = a[7];	// 飞行相时间

		double time_step = KINEMATICS_RES;
		double t_pre_success = 0;

		// 遍历飞行相时间，检查飞行相阶段的轨迹是否均为有效状态
		for (double t = 0; t < t_f; t += time_step) {
			State s_check = applyFlight(s, -t);	// 根据结束状态和飞行相时间，计算得到前一个状态

			if (isValidState(s_check, terrain, FLIGHT) == false) {	// 计算出的状态无效
				if (KINEMATICS_RES - 0.01 <= time_step and time_step <= KINEMATICS_RES + 0.01)	// 当前步长接近分辨率，即动作失败
					return false;
				else {
					time_step = KINEMATICS_RES;
					t = t_pre_success;
				}
			} else {
				time_step += KINEMATICS_RES;
				t_pre_success = t;
			}
		}

		State s_takeoff = applyFlight(s, -t_f);	// 计算抬腿时刻的状态

		// 自适应步长参数
		time_step = KINEMATICS_RES;
		t_pre_success = 0;

		// 遍历整个支撑相时间，检查支撑相阶段的轨迹是否均为有效状态，同时记录所能到达的最远有效状态到s_new
		for (double t = t_s; t >= 0; t -= time_step) {
			// 根据抬腿时刻的状态与给定动作，计算出前一个状态
			State s_check = applyStanceReverse(s_takeoff, a, t);

			if (isValidState(s_check, terrain, STANCE) == false) {	// 计算出的状态无效

				if (KINEMATICS_RES - 0.01 <= time_step and time_step <= KINEMATICS_RES + 0.01) {	// 当前步长接近分辨率
					// 将计算出来的到达无效状态的轨迹回退指定比例，记录此时到达的有效状态
					s_new = applyStance(s, a, t + BACKUP_RATIO * (t_s - t));
					return false;
				} else {	// 当前步长大于分辨率，可能因为步长过长而走到了障碍物处，减小步长
					time_step = KINEMATICS_RES;
					t = t_pre_success;
				}
			} else {	// 计算出的状态是有效的，记录此时到达的有效状态
				s_new = s_check;
				t_new = t_s - t;

				// 记录自适应步长参数
				time_step += KINEMATICS_RES;
				t_pre_success = t;
			}
		}

		// Check the exact starting state
		State s_start = applyStanceReverse(s_takeoff, a);	// 支撑相的初始状态，即为距离结束状态最远的一个状态
		if (isValidState(s_start, terrain, STANCE) == false)	// 若初始状态无效，则只记录支撑相最远能够到达的有效状态
			return false;
		else {	// 初始状态有效，记录此时的初始状态
			s_new = s_start;
			t_new = t_s;
		}

		// 站立相和飞行相轨迹完全有效
		return true;
	}

	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
	// s: 树中状态节点，即为结束状态
	// a: 给定动作
	// terrain: 高程图
	// s_new: 在给定结束状态和动作的情况下，遍历移动时间所能够走到的最远的有效状态
	// t_new: 到达s_new的时间
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain, State &s_new, double &t_new) {
		double t_s = a[6];	// 支撑相时间
		double t_f = a[7];	// 飞行相时间

		// 遍历飞行相时间，检查飞行相阶段的轨迹是否均为有效状态
		for (double t = 0; t < t_f; t += KINEMATICS_RES) {
			State s_check = applyFlight(s, -t);	// 根据结束状态和飞行相时间，计算得到前一个状态

			if (isValidState(s_check, terrain, FLIGHT) == false)	// 前一个状态无效，直接抛弃整段飞行相轨迹
				return false;
		}

		State s_takeoff = applyFlight(s, -t_f);	// 计算抬腿时刻的状态

		// 遍历整个支撑相时间，检查支撑相阶段的轨迹是否均为有效状态，同时记录所能到达的最远有效状态到s_new
		for (double t = t_s; t >= 0; t -= KINEMATICS_RES) {
			// 根据抬腿时刻的状态与给定动作，计算出前一个状态
			State s_check = applyStanceReverse(s_takeoff, a, t);

			if (isValidState(s_check, terrain, STANCE) == false) {	// 计算出的状态无效
				s_new = applyStance(s, a, t + BACKUP_RATIO * (t_s - t));	// 将计算出来的到达无效状态的轨迹回退指定比例，记录此时到达的有效状态
				return false;
			} else {	// 计算出的状态是有效的，记录此时到达的有效状态
				s_new = s_check;
				t_new = t_s - t;
			}
		}

		// Check the exact starting state
		State s_start = applyStanceReverse(s_takeoff, a);	// 支撑相的初始状态，即为距离结束状态最远的一个状态
		if (isValidState(s_start, terrain, STANCE) == false)	// 若初始状态无效，则只记录支撑相最远能够到达的有效状态
			return false;
		else {	// 初始状态有效，记录此时的初始状态
			s_new = s_start;
			t_new = t_s;
		}

		// 站立相和飞行相轨迹完全有效
		return true;
	}

	// 给定结束状态与动作对，检查该状态动作对在给定支撑相与飞行相时间的运动中，所到达的状态是否均为有效状态，同时记录最远有效状态到s_new中
	// s: 树中状态节点，即为结束状态
	// a: 给定动作
	// terrain: 高程图
	// 返回值：站立相和飞行相轨迹是否全部有效
	bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap &terrain) {
		State dummy_state;
		double dummy_time;
		return isValidStateActionPairReverse(s, a, terrain, dummy_state, dummy_time);
	}

}
