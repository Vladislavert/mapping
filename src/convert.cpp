#include "convert.hpp"

/**
 * @brief Перевод из структуры данных в виде XYZ в Eigen::Vector3d
 * 
 * @param coord координаты в виде структуры (XYZ)
 * @param vector3d 
 */
void	convertXYZToVector3d(const Coordinate3d* coord, Eigen::Vector3d& vector3d)
{
	vector3d[0] = coord->x;
	vector3d[1] = coord->y;
	vector3d[2] = coord->z;
}

/**
 * @brief 
 * 
 * @param vector3d 
 * @param coord 
 */
void	convertVector3dToXYZ(const Eigen::Vector3d& vector3d, Coordinate3d* coord)
{
	coord->x = vector3d[0];
	coord->y = vector3d[1];
	coord->z = vector3d[2];
}