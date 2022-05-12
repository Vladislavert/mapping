#ifndef MAPPING_DATA_TYPES_HPP_
#define MAPPING_DATA_TYPES_HPP_

#include "vector"


struct	Coordinate3d
{
	double	x;
	double	y;
	double	z;
};

/**
 * @brief карта
 * 
 * @param x позиция сетки по оси X в СК карты
 * @param y позиция сетки по оси Y в СК карты
 * @param z позиция сетки по оси Z в СК карты
 * @param isOccupied закрашен квадрат или нет
 */
struct	Map
{
	std::vector<double>	x;
	std::vector<double>	y;
	std::vector<double>	z;
	std::vector<double>	isOccupied;
};

struct	AngleEuler
{
	double	pitch; // перемещение по оси x
	double	roll; // перемещение по оси y
	double	yaw; // перемещение по оси z
};

struct	BuffData
{
	double**	x;
	double**	y;
	double**	z;
};

/**
 * @brief данные лидара
 * 
 * @param ranges массив расстояний
 * @param angleRay массив угловых положений лучей
 */
struct	DataLidar
{
	double*	ranges; // массив расстояний
	double*	angleRay; // массив угловых положения лучей
};

struct	ParallelepipedSize
{
	double	length; // длина
	double	width; // ширина
	double	height; // высота
};

typedef Eigen::Matrix<double, Eigen::Dynamic, 1>				VectorXd_t;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>	MatrixXd_t;

#endif // MAPPING_DATA_TYPES_HPP_
