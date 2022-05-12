#ifndef MAPPING_MAPPING_HPP_
#define MAPPING_MAPPING_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// для записи в файл
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "dataTypes.hpp"
#include "convert.hpp"
#include "math.hpp"
#include "filters.hpp"


class Mapping
{
public:		
		Mapping(ros::NodeHandle n);
		~Mapping() = default;

		void	publisherMapping();

		void	visualizationOccupancyGrid();
		void	saveMapInFile(char* fileName);

	private:
		ros::NodeHandle				n_; // объект, который хранит информацию о node, которая выполняется
		// ros::Publisher				pubMapping_; // публикуем карту
		ros::Subscriber				currentPoseLocal_; // подписываемся на текущую локальную позицию
		ros::Subscriber				currnetLaserScan_; // текущие данные с лидара
		ros::Publisher				markerPub_;

	    Eigen::Vector3d				currentPosition_; // положение ЛА  система координат на усмотрение разработчика
		Eigen::Quaterniond			orientationQuat_;
		sensor_msgs::LaserScan		laserScanValue_;
		unsigned int 				sizeRay_; // кол-во лучей, которые доходят до препятствия и принадлежат диапазону 
		Eigen::Vector3d				lidarInBodyCoord_; // положение лидара относительно СК тела
		AngleEuler					lidarInAngleBodyCoord_;
		Filters						filters_;
		visualization_msgs::Marker	points_; // точки, получаемые лидаром в СК карты
		visualization_msgs::Marker	parallelepipedGrid_; // параллелепипед, для заполнения карты
		geometry_msgs::Point		p_;
		double						timeSecsCurrent_;
		double						timeSecsPast_;
		double						dt_;
		double						secs_;
		unsigned int				countMeasurement_; // счётчик кол-ва измерений
		BuffData					buffData_; // буфер измеренных значений, кол-во которых задаётся переменной sizeBuffer.
											  // Требуется для медианного фильтра
		unsigned int				sizePointsPast_; // кол-во точек, который столкнулись с препятствием на предыдущем шаге
		unsigned int				sizeBuffer_; // размер буффера для хранения массива
		bool						startCalculate_; // запуск расчёта при подачи новых данных
		DataLidar					lidar_; // данные о лучах, которые встретились с препятствием

		// параметры occupancy grid
		// пользовательские настройки
		ParallelepipedSize			gridStep_; // шаг сетки по трём осям
		Coordinate3d				origin_; // начало координат сетки в СК карты
		Coordinate3d				mapSize_; // размер карты по трём осям(обновлять, так как динамически меняется)[м]

		Map							map_; // карта

		void			initNode();
		void			localPoseCallback(geometry_msgs::PoseStamped pose_);
		void			laserCallback(sensor_msgs::LaserScan msg);
		void			fillDataLaser();
		void			fillOccupancyGrid();
		unsigned int	buildMap(double x, double y, double z, const ParallelepipedSize& size, const Coordinate3d& originAxes);
		void			readDataLidar();
		Coordinate3d*	rangeToSensorCoord(const double* range);
		Coordinate3d*	sensorCoordToBodyCoord(const Coordinate3d* sensorCoord);
		Coordinate3d*	bodyCoordToMapCoord(Coordinate3d* bodyCoord);
		void			clearData(double secClearData);
		void			deleteArray2d(double** array, unsigned int arraySize);
		void			deleteBuffData(BuffData* data, unsigned int dataSize);
		void			allocationBuffData();
		bool			checkIntervalMinMax(double data, double min, double max);
		double			calculateMeanValueMap(const std::vector<double>& mapValue);
		
};

#endif // MAPPING_MAPPING_HPP_
