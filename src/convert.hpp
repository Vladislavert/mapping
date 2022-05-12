#ifndef MAPPING_CONVERT_HPP_
#define MAPPING_CONVERT_HPP_

#include "Eigen/Dense"
#include "dataTypes.hpp"

void	convertXYZToVector3d(const Coordinate3d* coord, Eigen::Vector3d& vector3d);
void	convertVector3dToXYZ(const Eigen::Vector3d& vector3d, Coordinate3d* coord);
void	convertQuaternionToAngleEuler();

#endif // MAPPING_CONVERT_HPP_
