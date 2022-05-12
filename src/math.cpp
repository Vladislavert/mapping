#include "math.hpp"

/**
 * @brief матрица поворотов
 * 
 * @param roll крен
 * @param pitch тангаж
 * @param yaw рысканье
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix3d	Math::rotationMatrix(double pitch, double roll, double yaw)// проверить подстановку roll, pitch, yaw
{
	Eigen::Matrix3d R;


	// R << 	c(pitch)*c(yaw),							s(pitch)*c(yaw), 							-s(yaw),
	// 		c(pitch)*s(roll)*s(yaw) - s(pitch)*c(roll),	s(pitch)*s(roll)*s(yaw) + c(pitch)*c(roll),	s(roll)*c(yaw),
	// 		c(pitch)*s(yaw)*c(roll) + s(pitch)*s(roll),	s(pitch)*c(roll)*s(yaw) - c(pitch)*s(roll),	c(roll)*c(yaw);
	R << 	c(yaw)*c(roll),							s(yaw)*c(roll), 							-s(roll),
			c(yaw)*s(pitch)*s(roll) - s(yaw)*c(pitch),	s(yaw)*s(pitch)*s(roll) + c(yaw)*c(pitch),	s(pitch)*c(roll),
			c(yaw)*s(roll)*c(pitch) + s(yaw)*s(pitch),	s(yaw)*c(pitch)*s(roll) - c(yaw)*s(pitch),	c(pitch)*c(roll);
	// R << 	c(yaw)*c(pitch),										-c(pitch)*s(yaw), 									s(pitch),
	// 		s(roll)*s(pitch)*c(yaw) - c(roll)*s(yaw),		-s(roll)*s(pitch)*s(yaw) + c(roll)*c(yaw),	s(roll)*c(pitch),
	// 		-c(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw),	c(roll)*s(pitch)*s(yaw) + s(roll)*c(yaw),	c(roll)*c(pitch);
	// R << 	c(pitch)*c(yaw),	s(roll)*s(pitch)*c(yaw) - c(roll)*s(yaw),	c(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw),
	// 		c(pitch)*s(yaw),	s(roll)*s(pitch)*s(yaw) + c(roll)*c(yaw),	c(roll)*s(pitch)*s(yaw) + s(roll)*c(yaw),
	// 		-s(pitch),				s(roll)*c(pitch),									c(roll)*c(pitch);
	return (R);
}

Eigen::Matrix3d	Math::rotationMatrix2d(double angle)
{
	Eigen::Matrix3d R;

	R << c(angle),	s(angle),	0,
		 -s(angle),	c(angle), 	0,
		 1, 		1, 			1;

	return (R);
}

// Eigen::Matrix2d	Math::rotationMatrix2d(double angle)
// {
// 	Eigen::Matrix2d R;

// 	R << c(angle),	s(angle),
// 		 -s(angle),	c(angle);

// 	return (R);
// }

/**
 * @brief возведение в квадрат
 * 
 * @param argument аргумент, который возводят в квадрат
 * @return значение аргумента в квадрате
 */
double			Math::squaring(const double &argument)
{
	return (argument * argument);
}

/**
 * @brief функция для нахождения косинуса
 * 
 * @param angl угол в радианах
 * @return значение косинуса
 */
double			Math::c(const double angl)
{
	return (cos(angl));
}

/**
 * @brief функция для нахождения синуса
 * 
 * @param angl угол в радианах
 * @return значение синуса
 */
double			Math::s(const double angl)
{
	return (sin(angl));
}

/**
 * @brief запись строки матрицы в VectorXd_t
 * 
 * @param matrix матрица
 * @param indexRows индекс строки, которую требуется записать в Vector3d
 * @return VectorXd_t из строки матрицы с индексом indexRows
 */
VectorXd_t		Math::matrixToVectorXd_t(MatrixXd_t matrix, unsigned int indexRows)
{
	VectorXd_t resultVectorXd_t(matrix.cols());

	for	(unsigned int i = 0; i < matrix.cols(); i++)
	{
		resultVectorXd_t(i) = matrix(indexRows, i);
	}

	return (resultVectorXd_t);
}
