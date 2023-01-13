#ifndef FAST_TERRAIN_MAP_H
#define FAST_TERRAIN_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <vector>
#include <fstream>
#include <iomanip>
#include <cmath>

//! A terrain map class built for fast and efficient sampling
/*!
   FastTerrainMap is a class built for lightweight and efficient sampling of the terrain for height and slope.
*/
class FastTerrainMap {
public:

	/**
	 * @brief Constructor for FastTerrainMap Class    构造函数
	 * @return Constructed object of type FastTerrainMap
	 */
	FastTerrainMap();

	/**
	 * @brief Load data from a grid_map::GridMap object into a FastTerrainMap object    直接给定地图数据
	 * @param[in] x_size  The number of elements in the x direction  x轴元素数目
	 * @param[in] y_size int The number of elements in the y direction  y轴元素数目
	 * @param[in] x_data std::vector<double> The vector of x data       x轴数据
	 * @param[in] y_data std::vector<double> The vector of y data       y轴数据
	 * @param[in] z_data std::vector<std::vector<double>> The nested vector of z data at each [x,y] location  (x,y)处z轴数据
	 * @param[in] std::vector<std::vector<double>> The nested vector of the x component of the gradient at each [x,y] location  (x,y)处梯度的x轴分量
	 * @param[in] std::vector<std::vector<double>> The nested vector of the y component of the gradient at each [x,y] location  (x,y)处梯度的y轴分量
	 * @param[in] std::vector<std::vector<double>> The nested vector of the z component of the gradient at each [x,y] location  (x,y)处梯度的z轴分量
	 */
	void loadData(int x_size, int y_size, std::vector<double> x_data, std::vector<double> y_data,
				  std::vector <std::vector<double>> z_data, std::vector <std::vector<double>> dx_data,
				  std::vector <std::vector<double>> dy_data, std::vector <std::vector<double>> dz_data);

	/**
	 * @brief Load data from a grid_map::GridMap object into a FastTerrainMap object    从grid_map::GridMap对象中加载地图数据
	 * @param[in] grid_map::GridMap object with map data
	 */
	void loadDataFromGridMap(grid_map::GridMap map);

	/**
	 * @brief Return the ground height at a requested location  根据双线性插值计算给定位置(x,y)处的地面高度
	 * @param[in] double x location
	 * @param[in] double y location
	 * @return double ground height at location [x,y]
	 */
	double getGroundHeight(const double x, const double y);

	/**
	 * @brief 根据双线性插值判断给定位置(x,y)处的地面高度是否是 nan
	 * @param[in] double x location
	 * @param[in] double y location
	 * @return double ground height at location [x,y]
	 */
	bool heightIsNan(const double x, const double y);

	/**
	 * @brief Return the surface normal at a requested location 返回给定位置(x,y)处的表面法线，需要地图梯度信息
	 * @param[in] double x location
	 * @param[in] double y location
	 * @return std::array<double, 3> surface normal at location [x,y]
	 */
	std::array<double, 3> getSurfaceNormal(const double x, const double y);

	/**
	 * @brief Return the vector of x_data of the map  返回x轴数据
	 * @return std::vector<double> of x locations in the grid
	 */
	std::vector<double> getXData();

	/**
	 * @brief Return the vector of y_data of the map  返回y轴数据
	 * @return std::vector<double> of y locations in the grid
	 */
	std::vector<double> getYData();

	/**
	 * @brief 保存地图的xyz数据到文件，注意，删除后会抛出异常，防止多次重复保存
	 * @param[in] filePath 保存文件夹路径
	 */
	void saveData(const std::string &dirPath);

	/**
	 * @brief 将给定vector保存到指定路径
	 * @param[in] filePath 保存文件路径
	 * @param[in] data 待保存数据
	 */
	void saveDataToTxt(const std::string &filePath, const std::vector<double> &data);
	void saveDataToTxt(const std::string &filePath, const std::vector <std::vector<double>> &data);

private:

	/// The number of elements in the x direction x轴方向元素数目
	int x_size_;

	/// The number of elements in the y direction y轴方向元素数目
	int y_size_;

	/// The vector of x data  x轴数据
	std::vector<double> x_data_;

	/// The vector of y data  y轴数据
	std::vector<double> y_data_;

	/// The nested vector of z data at each [x,y] location  z轴数据
	std::vector <std::vector<double>> z_data_;

	/// The nested vector of the x component of the gradient at each [x,y] location   梯度的x轴分量
	std::vector <std::vector<double>> dx_data_;

	/// The nested vector of the y component of the gradient at each [x,y] location   梯度的y轴分量
	std::vector <std::vector<double>> dy_data_;

	/// The nested vector of the z component of the gradient at each [x,y] location   梯度的z轴分量
	std::vector <std::vector<double>> dz_data_;

};

#endif // FAST_TERRAIN_MAP_H
