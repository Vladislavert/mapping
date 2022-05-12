#ifndef MAPPING_MATH_HPP_
#define MAPPING_MATH_HPP_

#include "Eigen/Dense"
#include "dataTypes.hpp"

class Math
{
	public:
		static Eigen::Matrix3d	rotationMatrix(double roll, double pitch, double yaw);
		static Eigen::Matrix3d	rotationMatrix2d(double angle);
		static double			squaring(const double &argument);
		static double			c(const double angl);
		static double			s(const double angl);
		static VectorXd_t		matrixToVectorXd_t(MatrixXd_t matrix, unsigned int indexRows);
};

#endif // MAPPING_MATH_HPP_
