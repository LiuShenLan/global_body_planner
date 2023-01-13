#include "global_body_planner/terrain_map_publisher.h"

TerrainMapPublisher::TerrainMapPublisher(ros::NodeHandle nh) : terrain_map_(
		grid_map::GridMap({"elevation", "dx", "dy", "dz"})) {
	nh_ = nh;

	// Load rosparams from parameter server
	std::string terrain_map_topic, image_topic;

	nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");	// 高程图话题
	nh.param<std::string>("map_frame", map_frame_, "/map");	// map frame
	nh.param<double>("terrain_map_publisher/update_rate", update_rate_, 10);	// 收发数据的频率，当发布者在回调函数中被调用之后不再使用
	nh.param<std::string>("terrain_map_publisher/map_data_source", map_data_source_, "internal");	// 高程图数据源类型
	nh.param<std::string>("terrain_map_publisher/terrain_type", terrain_type_, "slope");	// 从csv文件中加载时的高程图

	// Setup pubs and subs
	terrain_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(terrain_map_topic, 1);	// 高程图发布者

	// Add image subscriber if data source requests an image 当数据源类型为image时添加图像数据订阅者
	if (map_data_source_.compare("image") == 0) {
		nh_.param<std::string>("topics/image", image_topic, "/image_publisher/image");
		nh_.param<double>("terrain_map_publisher/resolution", resolution_, 0.2);	// 地图分辨率
		nh_.param<double>("terrain_map_publisher/min_height", min_height_, 0.0);	// 地图最低高度
		nh_.param<double>("terrain_map_publisher/max_height", max_height_, 1.0);	// 地图最高高度
		image_sub_ = nh_.subscribe(image_topic, 1, &TerrainMapPublisher::loadMapFromImage, this);	// 当数据源类型为image时，图像数据订阅者
	}

	// Initialize the elevation layer on the terrain map	设置高程图的基本层
	terrain_map_.setBasicLayers({"elevation", "dx", "dy", "dz"});

}

// 使用xyz_data创建地图
void TerrainMapPublisher::createOwnMap() {
	// 参数设置，需要确保网格中心点与数据点对齐
	double res = 0.05;	// 分辨率
	int x_size = 221, y_size = 161;					// xy轴数据数目
	double x_start = -0.5, y_start = -4.0;			// xy轴最小坐标

	// 相关数据计算
	double x_end = x_start + res * (x_size - 1);	// x轴最大坐标
	double y_end = y_start + res * (y_size - 1);	// y轴最大坐标
	double x_length = x_end - x_start + res;		// x轴长度
	double y_length = y_end - y_start + res;		// y轴长度

	// 创建x_data
	std::vector<double> x_data_line(x_size);
	double x_val = x_start;
	for (int i = 0; i < x_size; ++i) {
		x_data_line[i] = round(x_val * 100) / 100;
		x_val = round((x_val + res) * 100) / 100;
	}
	std::vector<std::vector<double>> x_data(y_size, x_data_line);

	// 创建y_data
	std::vector<std::vector<double>> y_data;
	double y_val = y_start;
	for (int i = 0; i < y_size; ++i) {
		y_data.emplace_back(x_size, round(y_val * 100) / 100);
		y_val = round((y_val + res) * 100) / 100;
	}

	// 创建z_data
	std::vector<std::vector<double>> z_data(y_size, std::vector<double>(x_size, 0));

	// 修改z_data
	changeOwnMapZData(x_data, y_data, z_data);

	// 保存数据
	// saveData("/root/gbpl_ws/src/global_body_planner/data/test/x_data.csv", x_data);
	// saveData("/root/gbpl_ws/src/global_body_planner/data/test/y_data.csv", y_data);
	// saveData("/root/gbpl_ws/src/global_body_planner/data/test/z_data.csv", z_data);

	// Initialize the map	初始化地图
	terrain_map_.setFrameId(map_frame_);	// map frame
	terrain_map_.setGeometry(grid_map::Length(x_length, y_length), res,	// 设置高程图的几何形状，清除所有数据。
							 grid_map::Position(x_data[0].front() - 0.5 * res + 0.5 * x_length,
												y_data.front()[0] - 0.5 * res + 0.5 * y_length));
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", terrain_map_.getLength().x(),
			 terrain_map_.getLength().y(), terrain_map_.getSize()(0), terrain_map_.getSize()(1));

	// 删除多余图层
	terrain_map_.erase("dx");
	terrain_map_.erase("dy");
	terrain_map_.erase("dz");

	// Load in the elevation and slope data	加载高程数据与坡度数据
	for (grid_map::GridMapIterator iterator(terrain_map_); !iterator.isPastEnd(); ++iterator) {
		const grid_map::Index index(*iterator);
		grid_map::Position position;
		terrain_map_.getPosition(*iterator, position);
		terrain_map_.at("elevation", *iterator) = z_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
	}
}

// 修改创建的地图中的高程值
void TerrainMapPublisher::changeOwnMapZData(std::vector<std::vector<double>> &x_data,
											std::vector<std::vector<double>> &y_data,
											std::vector<std::vector<double>> &z_data) {
	// CHANGE: 在这里修改生成的地图
	// changeOwnMapZDataRectangle(x_data, y_data, z_data, 2.5, -1, 3.5, 1, NAN);

	// 全局添加噪声
	changeOwnMapZDataRectangleRandom(x_data, y_data, z_data, -DBL_MAX, -DBL_MAX, DBL_MAX, DBL_MAX, 0, 0.01);

	std::vector<std::vector<double>> change =
		// 	x1		y1		x2		y2		z		delta
		{{	8.13,	-4,		8.42,	4,		0.158,	0.01},
		{	8.42,	-4,		8.71,	4,		0.316,	0.01},
		{	8.71, 	-4, 	10.5, 	4, 		0.474,	0.01},

		{	0.75, 	-3.15, 	2.05, 	-2.35, 	0.6,	0.1},
		{	4.25, 	-2.4, 	5.4, 	-1.75, 	0.5,	0.05},
		{	2.9, 	-0.6, 	3.25, 	1.15, 	0.158,	0.01},
		{	4.9, 	-0.5, 	5.3, 	0.35, 	-0.3,	0.01},
		{	6.5, 	0.45, 	7.2, 	1.05, 	0.7,	0.07},
		{	0.65, 	2.95, 	1.15, 	3.75, 	0.3,	0.08},
		{	4.4, 	2.8, 	5.7, 	3.55, 	0.65,	0.04},
		{	7.5, 	-2.6, 	9.45, 	-1.15, 	0.68,	0.06},
		{	6.2, 	1.1, 	9.2, 	2.3, 	-0.2,	0.06}

		};
	for (int i = 0; i < change.size(); ++i)
		changeOwnMapZDataRectangleRandom(x_data, y_data, z_data, change[i][0], change[i][1], change[i][2], change[i][3], change[i][4], change[i][5]);

	// addGaussianToVector(z_data, 0, 0.01);
}

// 修改data中(x1, y1)到(x2, y2)矩形区域中的值为val，矩形边界上的值也会被修改
void TerrainMapPublisher::changeOwnMapZDataRectangle(std::vector<std::vector<double>> &x_data,
													 std::vector<std::vector<double>> &y_data,
													 std::vector<std::vector<double>> &z_data,
													 double x1, double y1, double x2, double y2, double val) {
	if (x1 > x_data.front().back() or x2 < x_data.front().front() or	// x超出边界
		y1 > y_data.back().front() or y2 < y_data.front().front() or	// y超出边界
		x1 >= x2 or y1 >= y2)	// 矩形无面积
		return;

	// 待修改范围索引
	int x_index1, y_index1, x_index2, y_index2;
	findXYIndex(x_data, y_data, x1, y1, x2, y2, x_index1, y_index1, x_index2, y_index2);

	for (int i = y_index1; i < y_index2; ++i)
		for (int j = x_index1; j < x_index2; ++j)
			z_data[i][j] = val;
}

// 修改data中(x1, y1)到(x2, y2)矩形区域中的值为均值为val，三倍标准差为delta的高斯分布随机值，矩形边界上的值也会被修改
void TerrainMapPublisher::changeOwnMapZDataRectangleRandom(std::vector<std::vector<double>> &x_data,
													 std::vector<std::vector<double>> &y_data,
													 std::vector<std::vector<double>> &z_data,
													 double x1, double y1, double x2, double y2,
													 double mu, double delta) {
	if (x1 > x_data.front().back() or x2 < x_data.front().front() or	// x超出边界
		y1 > y_data.back().front() or y2 < y_data.front().front() or	// y超出边界
		x1 >= x2 or y1 >= y2)	// 矩形无面积
		return;

	// 待修改范围索引
	int x_index1, y_index1, x_index2, y_index2;
	findXYIndex(x_data, y_data, x1, y1, x2, y2, x_index1, y_index1, x_index2, y_index2);

	// 均值与标准差
	double val_min = mu - delta, val_max = mu + delta;
	std::default_random_engine generator(time(0));	// 随机数生成器
	std::normal_distribution<double> gaussian(mu, delta);	// 高斯分布，高斯分布99.7%落在 3*delta 内
	double val = gaussian(generator);	// 生成高斯分布

	for (int i = y_index1; i < y_index2; ++i) {
		for (int j = x_index1; j < x_index2; ++j) {
			val = gaussian(generator);
			while (val < val_min or val > val_max)
				val = gaussian(generator);
			z_data[i][j] = val;
		}
	}
}

// 根据坐标值x_data与y_data，寻找矩形区间[x1, y1]到[x2, y2]所对应的矩形索引区间[x_index1, y_index1]到[x_index2, y_index2]
void TerrainMapPublisher::findXYIndex(std::vector<std::vector<double>> &x_data,
				std::vector<std::vector<double>> &y_data,
				double x1, double y1, double x2, double y2,
				int& x_index1, int& y_index1, int& x_index2, int& y_index2) {
	int x_size = x_data.front().size();	// x轴数据数目
	int y_size = x_data.size();			// y轴数据数目

	// 寻找x轴索引
	if (x1 <= x_data.front().front())
		x_index1 = 0;
	else {
		for (int i = 0; i < x_size - 1; ++i) {
			if (x_data.front()[i] <= x1 and x1 < x_data.front()[i + 1]) {
				x_index1 = i;
				break;
			}
		}
	}
	if (x2 >= x_data.front().back())
		x_index2 = x_data.front().size();
	else {
		for (int i = x_size - 1; i > 0; --i) {
			if (x_data.front()[i - 1] <= x2 and x2 < x_data.front()[i]) {
				x_index2 = i;
				break;
			}
		}
	}


	// 寻找y轴索引
	if (y1 <= y_data.front().front())
		y_index1 = 0;
	else {
		for (int i = 0; i < y_size - 1; ++i) {
			if (y_data[i].front() <= y1 and y1 < y_data[i + 1].front()) {
				y_index1 = i;
				break;
			}
		}
	}
	if (y2 >= y_data.back().front())
		y_index2 = y_data.size();
	else {
		for (int i = y_size - 1; i > 0; --i) {
			if (y_data[i - 1].front() <= y2 and y2 < y_data[i].front()) {
				y_index2 = i;
				break;
			}
		}
	}
}

// 向指定数组中添加高斯噪声
void TerrainMapPublisher::addGaussianToVector(std::vector<std::vector<double>> &data, double mu, double delta) {
	std::default_random_engine generator(time(0));	// 随机数生成器
	std::normal_distribution<double> gaussian(mu, delta);	// 高斯分布

	// 添加随机数
	for (int i = 0; i < data.size(); ++i)
		for (int j = 0; j < data[i].size(); ++j)
			if (!std::isnan(data[i][j]))
				data[i][j] += gaussian(generator);
}

// 打印vector数据到文件
void TerrainMapPublisher::saveData(const std::string &file_path, std::vector<std::vector<double>> &data) {
	std::ofstream file(file_path);
	file.setf(std::ios::fixed | std::ios::right);	// 不使用科学计数法，右对齐
	file.precision(2);	// 保留小数位数
	for (int i = 0; i < data.size(); ++i) {
		file << std::setw(5) << data[i].front();	// 输出宽度为5字符
		for (int j = 1; j < data[i].size(); ++j)
			file << ',' << std::setw(5) << data[i][j];
		file << std::endl;
	}
	file.close();
}

// 创建地图
void TerrainMapPublisher::createMap() {
	// Set initial map parameters and geometry
	terrain_map_.setFrameId(map_frame_);
	terrain_map_.setGeometry(grid_map::Length(12.0, 5.0), 0.2, grid_map::Position(4.0, 0.0));
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", terrain_map_.getLength().x(),
			 terrain_map_.getLength().y(), terrain_map_.getSize()(0), terrain_map_.getSize()(1));

	// Add an obstacle
	double obs_center[] = {2, 0};
	double obs_radius = 0.5;
	for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {

		grid_map::Position position;
		terrain_map_.getPosition(*it, position);
		double x_diff = position.x() - obs_center[0];
		double y_diff = position.y() - obs_center[1];

		if (x_diff * x_diff + y_diff * y_diff <= obs_radius * obs_radius) {
			terrain_map_.at("elevation", *it) = 0.1;
		} else {
			terrain_map_.at("elevation", *it) = 0.0;
		}

		terrain_map_.at("dx", *it) = 0.0;
		terrain_map_.at("dy", *it) = 0.0;
		terrain_map_.at("dz", *it) = 1.0;
	}
}

// 从指定csv文件中加载数据
std::vector<std::vector<double>> TerrainMapPublisher::loadCSV(std::string filename) {
	std::vector<std::vector<double>> data;
	std::ifstream inputFile(filename);
	int l = 0;

	while (inputFile) {
		l++;
		std::string s;	// 一行字符串数据
		if (!getline(inputFile, s))	// 数据读取失败
			break;

		if (s[0] != '#') {	// 跳过注释行
			std::istringstream ss(s);
			std::vector<double> record;

			while (ss) {
				std::string num;
				if (!getline(ss, num, ','))
					break;

				try {
					record.push_back(stod(num));
				} catch (const std::invalid_argument e) {	// 非数字，抛出异常，不过stod("nan")是可以正常push进record的
					std::cout << "NaN found in file " << filename << " line " << l << std::endl;
					e.what();
				}
			}
			data.push_back(record);
		}
	}

	if (!inputFile.eof()) {
		std::cerr << "Could not read file " << filename << "\n";
		std::__throw_invalid_argument("File not found.");
	}

	return data;
}

// 从csv文件中加载高程图
void TerrainMapPublisher::loadMapFromCSV() {

	// Load in all terrain data	加载所有高程图数据
	std::string package_path = ros::package::getPath("global_body_planner");
	std::vector<std::vector<double>> x_data = loadCSV(package_path + "/data/" + terrain_type_ + "/xdata.csv");
	std::vector<std::vector<double>> y_data = loadCSV(package_path + "/data/" + terrain_type_ + "/ydata.csv");
	std::vector<std::vector<double>> z_data = loadCSV(package_path + "/data/" + terrain_type_ + "/zdata.csv");
	std::vector<std::vector<double>> dx_data = loadCSV(package_path + "/data/" + terrain_type_ + "/dxdata.csv");
	std::vector<std::vector<double>> dy_data = loadCSV(package_path + "/data/" + terrain_type_ + "/dydata.csv");
	std::vector<std::vector<double>> dz_data = loadCSV(package_path + "/data/" + terrain_type_ + "/dzdata.csv");

	// Grab map length and resolution parameters, make sure resolution is square (and align grid centers with data points)
	// 高程图长度与分辨率，需要确保x轴分辨率与y轴分辨率相等，并且网格中心点与数据点对齐
	int x_size = z_data[0].size();	// x轴数据数目
	int y_size = z_data.size();		// y轴数据数目
	float x_res = x_data[0][1] - x_data[0][0];  // x轴分辨率
	float y_res = y_data[1][0] - y_data[0][0];  // y轴分辨率
	double x_length = x_data[0].back() - x_data[0].front() + x_res; // x轴长度
	double y_length = y_data.back()[0] - y_data.front()[0] + y_res; // y轴长度
	if (x_res != y_res)	// x轴分辨率不等于y轴分辨率
		throw std::runtime_error("Map did not have square elements, make sure x and y resolution are equal.");

	// Initialize the map	初始化地图
	terrain_map_.setFrameId(map_frame_);	// map frame
	terrain_map_.setGeometry(grid_map::Length(x_length, y_length), x_res,	// 设置高程图的几何形状，清除所有数据。
							 grid_map::Position(x_data[0].front() - 0.5 * x_res + 0.5 * x_length,
												y_data.front()[0] - 0.5 * y_res + 0.5 * y_length));
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", terrain_map_.getLength().x(),
			 terrain_map_.getLength().y(), terrain_map_.getSize()(0), terrain_map_.getSize()(1));

	// Load in the elevation and slope data	加载高程数据与坡度数据
	for (grid_map::GridMapIterator iterator(terrain_map_); !iterator.isPastEnd(); ++iterator) {
		const grid_map::Index index(*iterator);
		grid_map::Position position;
		terrain_map_.getPosition(*iterator, position);
		terrain_map_.at("elevation", *iterator) = z_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
		terrain_map_.at("dx", *iterator) = dx_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
		terrain_map_.at("dy", *iterator) = dy_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
		terrain_map_.at("dz", *iterator) = dz_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
	}
}

// 从图像数据中加载高程图
void TerrainMapPublisher::loadMapFromImage(const sensor_msgs::Image &msg) {

	// Initialize the map from the image message if not already done so	高程图未初始化时，从图像消息初始化地图
	if (!map_initialized_) {
		grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, terrain_map_);
		ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", terrain_map_.getLength().x(),
				 terrain_map_.getLength().y(), terrain_map_.getSize()(0), terrain_map_.getSize()(1));
		map_initialized_ = true;
	}

	// Add the data layers	添加高程图图层
	grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", terrain_map_, min_height_, max_height_);
	grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", terrain_map_);

	// Add in slope information	添加坡度信息
	for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {
		grid_map::Position position;
		terrain_map_.at("dx", *it) = 0.0;
		terrain_map_.at("dy", *it) = 0.0;
		terrain_map_.at("dz", *it) = 1.0;
	}

	// Move the map to place starting location at (0,0)	移动地图使得初始位置为(0, 0)
	grid_map::Position offset = {4.5, 0.0};
	terrain_map_.setPosition(offset);
}

// 发布地图消息
void TerrainMapPublisher::publishMap() {
	// Set the time at which the map was published	设置地图发布的时间
	ros::Time time = ros::Time::now();
	terrain_map_.setTimestamp(time.toNSec());

	// Generate grid_map message, convert, and publish	生成高程图消息、转换并发布
	grid_map_msgs::GridMap terrain_map_msg;
	grid_map::GridMapRosConverter::toMessage(terrain_map_, terrain_map_msg);
	terrain_map_pub_.publish(terrain_map_msg);
}

void TerrainMapPublisher::spin() {
	ros::Rate r(update_rate_);

	// Either wait for an image to show up on the topic or create a map from scratch	创建地图
	if (map_data_source_.compare("image") == 0) {	// 从图像消息创建地图
		// Spin until image message has been received and processed	自旋直到图像消息被接收和处理
		boost::shared_ptr < sensor_msgs::Image const> shared_image;
		while ((shared_image == nullptr) && ros::ok()) {
			shared_image = ros::topic::waitForMessage<sensor_msgs::Image>("/image_publisher/image", nh_);
			ros::spinOnce();
		}
	} else if (map_data_source_.compare("csv") == 0)	// 读取csv文件中的地图
		loadMapFromCSV();
	else if (map_data_source_.compare("create") == 0)	// 使用createOwnMap()函数创建地图
		createOwnMap();
	else	// default
		createMap();

	// Continue publishing the map at the update rate	按照更新频率发布地图消息
	while (ros::ok()) {
		publishMap();	// 发布地图消息
		ros::spinOnce();
		r.sleep();
	}
}
