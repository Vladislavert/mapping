#ifndef CONVERT_HPP
#define CONVERT_HPP

#include "Eigen/Dense"
#include "typeData.hpp"

void	convertXYZToVector3d(const Coordinate3d* coord, Eigen::Vector3d& vector3d);
void	convertVector3dToXYZ(const Eigen::Vector3d& vector3d, Coordinate3d* coord);
void	convertQuaternionToAngleEuler();

#endif