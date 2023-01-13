#ifndef TERRAIN_MAP_PUBLISHER_H
#define TERRAIN_MAP_PUBLISHER_H

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/package.h>

#include <string>
#include <vector>
#include <sstream>	//istringstream
#include <iostream>	// cout
#include <fstream>	// ifstream
#include <random>

//! A terrain map publishing class
/*!
   TerrainMapPublisher is a class for publishing terrain maps from a variety of sources, including from scratch.
   从各种来源发布高程图的class
*/
class TerrainMapPublisher {
public:
	/**
	 * @brief Constructor for TerrainMapPublisher Class		构造函数
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type TerrainMapPublisher
	 */
	TerrainMapPublisher(ros::NodeHandle nh);

	/**
	 * @brief 使用xyz_data创建地图
	 */
	void createOwnMap();

	/**
	 * @brief 修改创建的地图中的高程值
	 * @param[in] z_data 待修改高程值vector
	 */
	void changeOwnMapZData(std::vector<std::vector<double>> &x_data, std::vector<std::vector<double>> &y_data,
						   std::vector<std::vector<double>> &z_data);

	/**
	 * @brief 修改data中(x1, y1)到(x2, y2)矩形区域中的值为val，矩形边界上的值也会被修改
	 * @param[in] x_data x坐标
	 * @param[in] y_data y坐标
	 * @param[in] z_data 待修改vector
	 * @param[in] x1 矩形区域x坐标较小值
	 * @param[in] y1 矩形区域y坐标较小值
	 * @param[in] x2 矩形区域x坐标较大值
	 * @param[in] y2 矩形区域y坐标较大值
	 * @param[in] val 修改后的值
	 */
	void changeOwnMapZDataRectangle(std::vector<std::vector<double>> &x_data,
									std::vector<std::vector<double>> &y_data,
									std::vector<std::vector<double>> &z_data,
									double x1, double y1, double x2, double y2, double val);

	/**
	 * @brief 修改data中(x1, y1)到(x2, y2)矩形区域中的值为均值为val，三倍标准差为delta的高斯分布随机值，矩形边界上的值也会被修改
	 * @param[in] x_data x坐标
	 * @param[in] y_data y坐标
	 * @param[in] z_data 待修改vector
	 * @param[in] x1 矩形区域x坐标较小值
	 * @param[in] y1 矩形区域y坐标较小值
	 * @param[in] x2 矩形区域x坐标较大值
	 * @param[in] y2 矩形区域y坐标较大值
	 * @param[in] mu 高斯分布均值
	 * @param[in] delta 高斯分布标准差的三倍
	 */
	void changeOwnMapZDataRectangleRandom(std::vector<std::vector<double>> &x_data,
									std::vector<std::vector<double>> &y_data,
									std::vector<std::vector<double>> &z_data,
									double x1, double y1, double x2, double y2,
									double mu, double delta);

	/**
	 * @brief 根据坐标值x_data与y_data，寻找矩形区间[x1, y1]到[x2, y2]所对应的矩形索引区间[x_index1, y_index1]到[x_index2, y_index2]
	 * @param[in] x_data x坐标
	 * @param[in] y_data y坐标
	 * @param[in] x1 矩形区域x坐标较小值
	 * @param[in] y1 矩形区域y坐标较小值
	 * @param[in] x2 矩形区域x坐标较大值
	 * @param[in] y2 矩形区域y坐标较大值
	 * @param[out] x_index1 矩形区域x索引较小值
	 * @param[out] y_index1 矩形区域y索引较小值
	 * @param[out] x_index2 矩形区域x索引较大值
	 * @param[out] y_index2 矩形区域y索引较大值
	 */
	void findXYIndex(std::vector<std::vector<double>> &x_data,
									std::vector<std::vector<double>> &y_data,
									double x1, double y1, double x2, double y2,
									int& x_index1, int& y_index1, int& x_index2, int& y_index2);
	/**
	 * @brief 向指定数组中添加高斯噪声
	 * @param[in] data 待修改数组
	 * @param[in] mu 高斯噪声均值
	 * @param[in] delta 高斯噪声标准差
	 */
	void addGaussianToVector(std::vector<std::vector<double>> &data, double mu, double delta);

	/**
	 * @brief 打印vector数据到文件
	 * @param[in] file_path 保存文件路径
	 * @param[in] data 待打印vector
	 */
	void saveData(const std::string &file_path, std::vector<std::vector<double>> &data);

	/**
	 * @brief Creates the map object from scratch	创建地图
	 */
	void createMap();

	/**
	 * @brief Loads data from a specified CSV file into a nested std::vector structure	从指定csv文件中加载数据
	 * @param[in] filename Path to the CSV file
	 * @return Data from the CSV in vector structure
	 */
	std::vector <std::vector<double>> loadCSV(std::string filename);

	/**
	 * @brief Loads data into the map object from a CSV	从csv文件中加载高程图
	 */
	void loadMapFromCSV();

	/**
	 * @brief Loads data into the map object from an image topic	从图像数据中加载高程图
	 * @param[in] msg ROS image message
	 */
	void loadMapFromImage(const sensor_msgs::Image &msg);

	/**
	 * @brief Publishes map data to the terrain_map topic	发布地图消息
	 */
	void publishMap();

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency	使用设定频率发布高程图
	 */
	void spin();

private:

	/// ROS Subscriber for image data	当数据源类型为image时，图像数据订阅者
	ros::Subscriber image_sub_;

	/// ROS Publisher for the terrain map	高程图发布者
	ros::Publisher terrain_map_pub_;

	/// Nodehandle to pub to and sub from	ros节点句柄
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data, unused since pubs are called in callbacks	收发数据的频率，当发布者在回调函数中被调用之后不再使用
	double update_rate_;

	/// Handle for the map frame
	std::string map_frame_;

	/// GridMap object for terrain data	高程图grid_map::GridMap对象
	grid_map::GridMap terrain_map_;

	/// String of the source of the terrain map data	高程图数据源类型
	std::string map_data_source_;

	/// String of terrain type (if loading from csv)	从csv文件中加载时的高程图
	std::string terrain_type_;

	/// Bool to flag if the map has been initialized yet	地图是否已经初始化
	bool map_initialized_ = false;

	/// 当数据源类型为image时，地图的分辨率、最低高度和最高高度
	double resolution_;	/// Double for map resolution	地图分辨率
	double min_height_;	/// Double for map min height	地图最低高度
	double max_height_;	/// Double for map max height	地图最高高度
};

#endif // TERRAIN_MAP_PUBLISHER_H
