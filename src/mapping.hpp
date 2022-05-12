#ifndef MAPPING_HPP
#define MAPPING_HPP

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

#include "typeData.hpp"
#include "convert.hpp"
#include "math.hpp"
#include "filters.hpp"


class Mapping
{
public:
		ros::NodeHandle				n; // объект, который хранит информацию о node, которая выполняется
		ros::Publisher				pubMapping; // публикуем карту
		ros::Subscriber				currentPoseLocal; // подписываемся на текущую локальную позицию
		ros::Subscriber				currnetLaserScan; // текущие данные с лидара
		ros::Publisher				marker_pub;
		
		Mapping(ros::NodeHandle n);
		~Mapping() = default;
		void	publisherMapping();

		// для отладки
		void			visualizationOccupancyGrid();
		int				saveMapInFile(char* fileName);

	private:
	    Eigen::Vector3d				currentPosition; // положение ЛА  система координат на усмотрение разработчика по данному параметру осуществляется обратная связь
		Eigen::Quaterniond			orientationQuat;
		sensor_msgs::LaserScan		laserScanValue;
		unsigned int 				sizeRay; // кол-во лучей, которые доходят до препятствия и принадлежат диапазону 
		Eigen::Vector3d				lidarInBodyCoord; // положение лидара относительно СК тела
		AngleEuler					lidarInAngleBodyCoord;
		Math						math;
		Filters						filters;
		visualization_msgs::Marker	points; // точки, получаемые лидаром в СК карты
		visualization_msgs::Marker	parallelepipedGrid; // параллелепипед, для заполнения карты
		geometry_msgs::Point		p;
		double						timeSecsCurrent;
		double						timeSecsPast;
		double						dt;
		double						secs;
		unsigned int				countMeasurement; // считает кол-во измерений
		BuffData					buffData; // буфер измеренных значений, кол-во которых задаётся переменной sizeBuffer.
											  // Требуется для медианного фильтра
		unsigned int				sizePointsPast; // кол-во точек, который столкнулись с препятствием на предыдущем шаге
		unsigned int				sizeBuffer; // размер буффера для хранения массива
		bool						startCalculate; // запуск расчётом при подачи новых данных
		DataLidar					lidar; // данные о лучах, которые встретились с препятствием

		// параметры occupancy grid(подумать о переносе в другое место)
		// пользовательские настройки
		ParallelepipedSize			gridStep; // шаг сетки по трём осям
		Coordinate3d				origin; // начало координат сетки в СК карты
		Coordinate3d				mapSize; // размер карты по трём осям(обновлять, так как динамически меняется)[м]
		double						updateDistance; // дистанция на которой карта будет обновляться(по x и y обновление идёт одновременно)[м]
		double						updateDistanceHeight; // дистанция на которой карта будет обновляться в высоту
		// end

		// Coordinate3d*				arrayCoordinateGrid; // массив координат сетки
		Map							map; // карта

		void			initNode();
		void			localPoseCallback(geometry_msgs::PoseStamped pose_);
		// void	laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		void			laserCallback(sensor_msgs::LaserScan msg);
		void			fillDataLaser();
		void			fillOccupancyGrid(Coordinate3d& mapSize);
		// void			buildMap(double& x, double& y, double& z);
		unsigned int	buildMap(double& x, double& size, double& originAxes);
		unsigned int	buildMap(const double& x, const double& y, const double& z, const ParallelepipedSize& size, const Coordinate3d& originAxes);
		void			readDataLidar();
		Coordinate3d*	rangeToSensorCoord(const double* range);
		Coordinate3d*	sensorCoordToBodyCoord(const Coordinate3d* sensorCoord);
		Coordinate3d*	bodyCoordToMapCoord(Coordinate3d* bodyCoord);
		void			clearData(double secClearData);
		void			deleteArray2d(double** array, unsigned int arraySize);
		void			deleteBuffData(BuffData* data, unsigned int dataSize);
		void			allocationBuffData();
		double			saturation(const double& data, const double& min, const double& max);
		double			saturationMin(const double& data, const double& min);
		bool			checkIntervalMinMax(const double& data, const double& min, const double& max);
		// void			updateMap();
		double			calculateCorrelation(Map map, Map particle);
		double			calculateMeanValueMap(const std::vector<double>& mapValue);
		
};

#endif