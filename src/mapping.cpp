#include "mapping.hpp"

Mapping::Mapping(ros::NodeHandle n)
{
	n_ = n;
	lidar_.ranges = nullptr;
	lidar_.angleRay = nullptr;
	// инициализация положения лидара относительно СК БЛА
	lidarInBodyCoord_ << 0, 0, 0;
	lidarInAngleBodyCoord_.pitch = 0;
	lidarInAngleBodyCoord_.roll = 0;
	lidarInAngleBodyCoord_.yaw = 0;
	timeSecsCurrent_ = 0;
	timeSecsPast_ = ros::Time::now().toSec();
	secs_ = 0;
	dt_ = 0;
	countMeasurement_ = 0;
	sizeBuffer_ = 1;
	buffData_.x = nullptr;
	buffData_.y = nullptr;
	buffData_.z = nullptr;
	sizePointsPast_ = 0;
	startCalculate_ = false;

	// инициализация параметров сетки
	gridStep_.length = 0.2;
	gridStep_.width = 0.2;
	gridStep_.height = 0.2;
	// размер карты в метрах
	mapSize_.x = 100;
	mapSize_.y = 100;
	mapSize_.z = 20;

	origin_.x = -40;
	origin_.y = -40;
	origin_.z = -3;

	initNode();
	fillOccupancyGrid();
}

Mapping::~Mapping()
{
	deleteBuffData(&buffData_, sizeRay_);
	deleteDataLidar(&lidar_);
}


/**
 * @brief Публикация сообщения 
 * 
 */
void			Mapping::publisherMapping()
{
	Coordinate3d* sensorCoord;
	Coordinate3d* bodyCoord;
	Coordinate3d* mapCoord;

	// вызов обновления карты
	if (startCalculate_ == true)
	{
		fillDataLaser();
		if (lidar_.ranges != nullptr && lidar_.angleRay != nullptr)
		{
			sensorCoord = rangeToSensorCoord(lidar_.ranges);
			bodyCoord = sensorCoordToBodyCoord(sensorCoord);
			mapCoord = bodyCoordToMapCoord(bodyCoord);
			parallelepipedGrid_.header.stamp = ros::Time::now();

			if (sizePointsPast_ != sizeRay_)
				countMeasurement_ = 0;
			if (countMeasurement_ == 0)
			{
				deleteBuffData(&buffData_, sizePointsPast_);
				allocationBuffData();
			}
			for	(unsigned int i = 0, j = 0; i < sizeRay_; i++)
			{
				buffData_.x[j][countMeasurement_] = mapCoord[i].x;
				buffData_.y[j][countMeasurement_] = mapCoord[i].y;
				buffData_.z[j][countMeasurement_] = mapCoord[i].z;
				j++;
			}
			countMeasurement_++;
			if (countMeasurement_ == sizeBuffer_)
			{
				for (unsigned int i = 0; i < sizeRay_; i++)
				{
					p_.x = filters_.median(buffData_.x[i], sizeBuffer_);
					p_.y = filters_.median(buffData_.y[i], sizeBuffer_);
					p_.z = filters_.median(buffData_.z[i], sizeBuffer_);
					unsigned int index = buildMap(p_.x, p_.y, p_.z, gridStep_, origin_);
					if (map_.isOccupied[index] == 1)
					{
						p_.x = map_.x[index];
						p_.y = map_.y[index];
						p_.z = map_.z[index];
						parallelepipedGrid_.points.push_back(p_);
					}
				}
				countMeasurement_ = 0;
			}
			markerPub_.publish(parallelepipedGrid_);
		}
	}
	startCalculate_ = false;
}

/**
 * @brief Инициализация ноды
 * 
 */
void			Mapping::initNode()
{
    currentPoseLocal_ = n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &Mapping::localPoseCallback, this);
	currnetLaserScan_ = n_.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1, &Mapping::laserCallback, this);
	markerPub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

	points_.header.frame_id = "map";
	points_.action = visualization_msgs::Marker::ADD;
	points_.pose.orientation.w = 0;
	points_.id = 0;
	points_.type = visualization_msgs::Marker::POINTS;

	points_.scale.x = 0.05;
  	points_.scale.y = 0.05;
	points_.scale.z = 0.05;
	points_.color.g = 1.0f;
  	points_.color.a = 1.0;

	parallelepipedGrid_.header.frame_id = "map";
	parallelepipedGrid_.action = visualization_msgs::Marker::ADD;
	parallelepipedGrid_.pose.orientation.w = 0;
	parallelepipedGrid_.id = 1;
	parallelepipedGrid_.type = visualization_msgs::Marker::CUBE_LIST;
	parallelepipedGrid_.scale.x = gridStep_.length;
  	parallelepipedGrid_.scale.y = gridStep_.width;
	parallelepipedGrid_.scale.z = gridStep_.height;
	parallelepipedGrid_.color.g = 1.0f;
  	parallelepipedGrid_.color.a = 1.0;
}

/**
 * @brief Заполнение векторов локальной позиции БЛА
 * 
 * @param pose_ сообщение о локальной позиции БЛА
 */
void			Mapping::localPoseCallback(geometry_msgs::PoseStamped pose_)
{
	currentPosition_[0] = pose_.pose.position.x;
	currentPosition_[1] = pose_.pose.position.y; 
	currentPosition_[2] = pose_.pose.position.z;
	
	orientationQuat_.w() = pose_.pose.orientation.w;
	orientationQuat_.x() = pose_.pose.orientation.x;  
	orientationQuat_.y() = pose_.pose.orientation.y;
	orientationQuat_.z() = pose_.pose.orientation.z;
}

void			Mapping::laserCallback(sensor_msgs::LaserScan msg)
{
	startCalculate_ = true;
	laserScanValue_ = msg;
}

/**
 * @brief заполнение данных с лидара
 * 
 */
void			Mapping::fillDataLaser()
{
	sizePointsPast_ = sizeRay_;
	sizeRay_ = 0;
	double	countAngle;

	countAngle = laserScanValue_.angle_min;
	for	(unsigned int i = 0; i < laserScanValue_.ranges.size(); i++)
		if (checkIntervalMinMax(laserScanValue_.ranges[i], laserScanValue_.range_min, laserScanValue_.range_max))
			sizeRay_++;
	lidar_.ranges = new double[sizeRay_];
	lidar_.angleRay = new double[sizeRay_];
	for	(unsigned int i = 0, j = 0; i < laserScanValue_.ranges.size(); i++)
	{
		countAngle += laserScanValue_.angle_increment;
		if (checkIntervalMinMax(laserScanValue_.ranges[i], laserScanValue_.range_min, laserScanValue_.range_max))
		{
			lidar_.ranges[j] = laserScanValue_.ranges[i];
			lidar_.angleRay[j] = countAngle;
			j++;
		}
	}
}

/**
 * @brief Функция для перевод дальности в СК связанной с телом  в координаты СК связанной с телом
 * 
 * @param range дальность в СК связанной с телом
 * @return координаты в СК сенсора(X_s, Y_s, Z_s)
 */
Coordinate3d*	Mapping::rangeToSensorCoord(const double* range)
{
	Coordinate3d* sensorCoord = new Coordinate3d[sizeRay_];

	for	(unsigned int i = 0; i < sizeRay_; i++)
	{
		sensorCoord[i].x = range[i] * cos(lidar_.angleRay[i]);
		sensorCoord[i].y = range[i] * sin(lidar_.angleRay[i]);
		sensorCoord[i].z = 0;
	}

	return (sensorCoord);
}

/**
 * @brief Перевод из системы координат датчика в систему координат тела
 * 
 * @param sensorCoord координаты в СК датчика(X_s, Y_s, Z_s)
 * @return координаты в СК тела(X_b, Y_b, Z_b)
 */
Coordinate3d*	Mapping::sensorCoordToBodyCoord(const Coordinate3d* sensorCoord)
{
	Eigen::Vector3d	sensorCoordVect3d;
	Eigen::Vector3d	bodyCoordVect3d;
	Coordinate3d* bodyCoord = new Coordinate3d[sizeRay_];

	for	(unsigned int i = 0; i < sizeRay_; i++)
	{
		convertXYZToVector3d(&sensorCoord[i], sensorCoordVect3d);
		bodyCoordVect3d = lidarInBodyCoord_ + Math::rotationMatrix(lidarInAngleBodyCoord_.pitch, lidarInAngleBodyCoord_.roll, lidarInAngleBodyCoord_.yaw).transpose() * sensorCoordVect3d;
		convertVector3dToXYZ(bodyCoordVect3d, &bodyCoord[i]);
	}

	return (bodyCoord);
}

/**
 * @brief Перевод из системы координат тела в систему координат карты
 * 
 * @param bodyCoord координаты в СК тела(X_b, Y_b, Z_b)
 * @return координаты в СК карты(X_map, Y_map, Z_map) 
 */
Coordinate3d*	Mapping::bodyCoordToMapCoord(Coordinate3d* bodyCoord)
{
	double r, p, y;
	Eigen::Vector3d	bodyCoordVect3d;
	Eigen::Vector3d	mapCoordVect3d;
	Coordinate3d* mapCoord = new Coordinate3d[sizeRay_];

	tf2::Quaternion q(orientationQuat_.x(), orientationQuat_.y(), orientationQuat_.z(), orientationQuat_.w());
	tf2::Matrix3x3 m(q);
	m.getRPY(r, p, y);

	for	(unsigned int i = 0; i < sizeRay_; i++)
	{
		convertXYZToVector3d(&bodyCoord[i], bodyCoordVect3d);
		mapCoordVect3d = currentPosition_  + Math::rotationMatrix(r, p, y).transpose() * bodyCoordVect3d;
		convertVector3dToXYZ(mapCoordVect3d, &mapCoord[i]);
	}

	return (mapCoord);
}

void			Mapping::fillOccupancyGrid()
{
	unsigned int length = round(mapSize_.x/gridStep_.length);
	unsigned int width = round(mapSize_.y/gridStep_.width);
	unsigned int height = round(mapSize_.z/gridStep_.height);

	double* arrayLength = new double[length]; // координаты центра сетки в длину
	double* arrayWidth = new double[width]; // координаты центра сетки в ширину
	double* arrayHeight = new double[height]; // координаты центра сетки в высоту

	arrayLength[0] = origin_.x + gridStep_.length / 2;
	arrayWidth[0] = origin_.y + gridStep_.width / 2;
	arrayHeight[0] = origin_.z + gridStep_.height / 2;
	for	(unsigned int i = 1; i < length; i++)
	{
		arrayLength[i] = arrayLength[i - 1] + gridStep_.length;
	}
	for	(unsigned int i = 1; i < width; i++)
	{
		arrayWidth[i] = arrayWidth[i - 1] + gridStep_.width;
	}
	for	(unsigned int i = 1; i < height; i++)
	{
		arrayHeight[i] = arrayHeight[i - 1] + gridStep_.height;
	}

	for	(unsigned int count = 0, i = 0, j = 0, k = 0; count < (length * width * height); count++)
	{
		map_.x.push_back(arrayLength[i]);
		map_.y.push_back(arrayWidth[j]);
		map_.z.push_back(arrayHeight[k]);
		map_.isOccupied.push_back(0);
		if (i < length - 1)
			i++;
		else
		{
			i = 0;
			if (j < width - 1)
				j++;
			else
			{
				j = 0;
				if (k < height - 1)
					k++;		
			}
		}
	}
	delete[] arrayLength;
	delete[] arrayWidth;
	delete[] arrayHeight;
}

unsigned int	Mapping::buildMap(double x, double y, double z, const ParallelepipedSize& size, const Coordinate3d& originAxes)
{
	unsigned int indexX;
	unsigned int indexY;
	unsigned int indexZ;
	unsigned int index;

	indexX = trunc((x - originAxes.x) / size.length);
	indexY = trunc((y - originAxes.y) / size.width);
	indexZ = trunc((z - originAxes.z) / size.height);

	index = indexX + (indexY * (mapSize_.x / size.length))
				   + (indexZ * (mapSize_.x / size.length) * (mapSize_.y / size.length));
	map_.isOccupied[index] = 1;

	return (index);
}

/**
 * @brief Вывод данных с лидара в консоль
 * 
 */
void	Mapping::readDataLidar()
{
	for (unsigned int i = 0; i < laserScanValue_.ranges.size(); i++)
	{
		ROS_INFO("DATA LIDAR: %f", laserScanValue_.ranges[i]);
	}
}

/**
 * @brief очистка данных о массиве точек, спустя время(secClearData)
 * 
 * @param secClearData время, через которое производится очистка данных[сек]
 */
void	Mapping::clearData(double secClearData)
{
	timeSecsCurrent_ = ros::Time::now().toSec();
	dt_ = timeSecsCurrent_ - timeSecsPast_;
	timeSecsPast_ = timeSecsCurrent_;
	secs_ += dt_;
	if (secs_ > secClearData)
	{
		points_.points.clear();
		secs_ = 0;
	}
}

/**
 * @brief очищает двумерный массив
 * 
 * @param array массив для удаления
 * @param arraySize размер массива
 */
void	Mapping::deleteArray2d(double** array, unsigned int arraySize)
{
	if (array != nullptr)
	{
		for(unsigned int i = 0; i < arraySize; i++)
			delete[] array[i];
		delete[] array;
	}
}

/**
 * @brief Очистка структуры, которая хранит в себе массив измерений по оси X, Y, Z
 * 
 * @param data структура массивов измерений (X, Y, Z)
 * @param dataSize кол-во измерений
 */
void	Mapping::deleteBuffData(BuffData* data, unsigned int dataSize)
{
	if (data->x != nullptr && data->y != nullptr && data->z != nullptr)
	{
		for(unsigned int i = 0; i < dataSize; i++)
		{
			delete[] data->x[i];
			delete[] data->y[i];
			delete[] data->z[i];
		}	
		delete[] data->x;
		delete[] data->y;
		delete[] data->z;
	}
}

void	Mapping::deleteDataLidar(DataLidar* data)
{
	if (data->angleRay != nullptr)
		delete[] data->angleRay;
	if (data->ranges != nullptr)
		delete[] data->ranges;
}

/**
 * @brief Выделение памяти для структуры  BuffData
 * 
 */
void	Mapping::allocationBuffData()
{
	buffData_.x = new double*[sizeRay_];
	buffData_.y = new double*[sizeRay_];
	buffData_.z = new double*[sizeRay_];
	for (unsigned int k = 0; k < sizeRay_; k++)
	{
		buffData_.x[k] = new double[sizeBuffer_];
		buffData_.y[k] = new double[sizeBuffer_];
		buffData_.z[k] = new double[sizeBuffer_];
	}
}

/**
 * @brief проверка на принадлежность данным интервалу от min до max
 * 
 * @param data данные, которые проверяются на принадлежность диапазону
 * @param min минимальное значение
 * @param max максимальное значение
 * @return true - принадлежит диапазону от min до max
 * @return false - не принадлежит диапазону от min до max
 */
bool	Mapping::checkIntervalMinMax(double data, double min, double max)
{
	bool result;

	result = false;
	if (data < max && data > min)
		result = true;

	return (result);
}

void			Mapping::saveMapInFile(char* fileName)
{
	std::ofstream file(fileName);

	file.clear();
	for	(unsigned int i = 0; i < map_.isOccupied.size(); i++)
		file << map_.x[i] << " " << map_.y[i] << " " << map_.z[i] << " " << map_.isOccupied[i] << "\n";
	file.close();
}
