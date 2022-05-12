#include "mapping.hpp"

Mapping::Mapping(ros::NodeHandle n)
{
	// initNode();
	this->n = n;
	lidar.ranges = nullptr;
	lidar.angleRay = nullptr;
	// инициализация положения лидара относительно СК БЛА
	lidarInBodyCoord << 0, 0, 0;
	lidarInAngleBodyCoord.pitch = 0;
	lidarInAngleBodyCoord.roll = 0;
	lidarInAngleBodyCoord.yaw = 0;
	timeSecsCurrent = 0;
	timeSecsPast = ros::Time::now().toSec();
	secs = 0;
	dt = 0;
	countMeasurement = 0;
	sizeBuffer = 1;
	buffData.x = nullptr;
	buffData.y = nullptr;
	buffData.z = nullptr;
	sizePointsPast = 0;
	startCalculate = false;

	// инициализация параметров сетки
	gridStep.length = 0.2;
	gridStep.width = 0.2;
	gridStep.height = 0.2;
	// размер карты в метрах
	mapSize.x = 100;
	mapSize.y = 100;
	mapSize.z = 20;
	// расстояние обновления карты
	updateDistance = 5;
	updateDistanceHeight = 5;

	origin.x = -40;
	origin.y = -40;
	origin.z = -3;

	initNode();

	fillOccupancyGrid(mapSize);
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
	// clearData(10);
	// readDataLidar();
	// visualizationOccupancyGrid();
	// вызов обновления карты
	if (startCalculate == true)
	{
		fillDataLaser();
		if (lidar.ranges != nullptr && lidar.angleRay != nullptr)
		{
			sensorCoord = rangeToSensorCoord(lidar.ranges);
			bodyCoord = sensorCoordToBodyCoord(sensorCoord);
			mapCoord = bodyCoordToMapCoord(bodyCoord);
			parallelepipedGrid.header.stamp = ros::Time::now();

			if (sizePointsPast != sizeRay)
				countMeasurement = 0;
			if (countMeasurement == 0)
			{
				deleteBuffData(&buffData, sizePointsPast);
				allocationBuffData();
			}
			for	(unsigned int i = 0, j = 0; i < sizeRay; i++)
			{
				buffData.x[j][countMeasurement] = mapCoord[i].x;
				buffData.y[j][countMeasurement] = mapCoord[i].y;
				buffData.z[j][countMeasurement] = mapCoord[i].z;
				j++;
			}
			countMeasurement++;
			if (countMeasurement == sizeBuffer)
			{
				for (unsigned int i = 0; i < sizeRay; i++)
				{
					p.x = filters.median(buffData.x[i], sizeBuffer);
					p.y = filters.median(buffData.y[i], sizeBuffer);
					p.z = filters.median(buffData.z[i], sizeBuffer);
					// временная переменная, которую стоит переместить в другое место
					unsigned int index = buildMap(p.x, p.y, p.z, gridStep, origin);
					if (map.isOccupied[index] == 1)
					{
						p.x = map.x[index];
						p.y = map.y[index];
						p.z = map.z[index];
						parallelepipedGrid.points.push_back(p);
					}
					// p.x = map[buildMap(p.x, gridStep.length, origin.x)].x;
					// p.y = map[buildMap(p.y, gridStep.width, origin.y)].y;
					// p.z = map[buildMap(p.z, gridStep.height, origin.z)].z;
					
				}
				double r_AB = calculateCorrelation(map, map);
				ROS_INFO("r_AB = %lf", r_AB);
				countMeasurement = 0;
			}
			marker_pub.publish(parallelepipedGrid);
		}
	}
	startCalculate = false;
}

/**
 * @brief Инициализация ноды
 * 
 */
void			Mapping::initNode()
{
    currentPoseLocal = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &Mapping::localPoseCallback, this);
	currnetLaserScan = n.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1, &Mapping::laserCallback, this);
	marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

	points.header.frame_id = "map";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;

	points.scale.x = 0.05;
  	points.scale.y = 0.05;
	points.scale.z = 0.05;
	points.color.g = 1.0f;
  	points.color.a = 1.0;

	parallelepipedGrid.header.frame_id = "map";
	parallelepipedGrid.action = visualization_msgs::Marker::ADD;
	parallelepipedGrid.pose.orientation.w = 0;
	parallelepipedGrid.id = 1;
	// parallelepipedGrid.type = visualization_msgs::Marker::parallelepipedGrid;
	// parallelepipedGrid.type = 6;
	parallelepipedGrid.type = visualization_msgs::Marker::CUBE_LIST;

	// parallelepipedGrid.scale.x = gridStep.length;
  	// parallelepipedGrid.scale.y = gridStep.width;
	// parallelepipedGrid.scale.z = gridStep.height;
	parallelepipedGrid.scale.x = gridStep.length;
  	parallelepipedGrid.scale.y = gridStep.width;
	parallelepipedGrid.scale.z = gridStep.height;
	parallelepipedGrid.color.g = 1.0f;
  	parallelepipedGrid.color.a = 1.0;
}

/**
 * @brief Заполнение векторов локальной позиции БЛА
 * 
 * @param pose_ сообщение о локальной позиции БЛА
 */
void			Mapping::localPoseCallback(geometry_msgs::PoseStamped pose_)
{
	currentPosition[0] = pose_.pose.position.x;
	currentPosition[1] = pose_.pose.position.y; 
	currentPosition[2] = pose_.pose.position.z;
	
	orientationQuat.w() = pose_.pose.orientation.w;
	orientationQuat.x() = pose_.pose.orientation.x;  
	orientationQuat.y() = pose_.pose.orientation.y;
	orientationQuat.z() = pose_.pose.orientation.z;
}

// создать стуктуру, которая будет описывать данные с датчика(дальность, угол)
void			Mapping::laserCallback(sensor_msgs::LaserScan msg)
{
	startCalculate = true;
	laserScanValue = msg;
}

/**
 * @brief заполнение данных с лидара
 * 
 */
void			Mapping::fillDataLaser()
{
	sizePointsPast = sizeRay;
	sizeRay = 0;
	double	countAngle;

	countAngle = laserScanValue.angle_min;
	for	(unsigned int i = 0; i < laserScanValue.ranges.size(); i++)
		if (checkIntervalMinMax(laserScanValue.ranges[i], laserScanValue.range_min, laserScanValue.range_max))
			sizeRay++;
	lidar.ranges = new double[sizeRay];
	lidar.angleRay = new double[sizeRay];
	for	(unsigned int i = 0, j = 0; i < laserScanValue.ranges.size(); i++)
	{
		countAngle += laserScanValue.angle_increment;
		if (checkIntervalMinMax(laserScanValue.ranges[i], laserScanValue.range_min, laserScanValue.range_max))
		{
			lidar.ranges[j] = laserScanValue.ranges[i];
			lidar.angleRay[j] = countAngle;
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
	Coordinate3d* sensorCoord = new Coordinate3d[sizeRay];

	for	(unsigned int i = 0; i < sizeRay; i++)
	{
		sensorCoord[i].x = range[i] * cos(lidar.angleRay[i]);
		sensorCoord[i].y = range[i] * sin(lidar.angleRay[i]);
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
	Coordinate3d* bodyCoord = new Coordinate3d[sizeRay];

	for	(unsigned int i = 0; i < sizeRay; i++)
	{
		convertXYZToVector3d(&sensorCoord[i], sensorCoordVect3d);
		bodyCoordVect3d = lidarInBodyCoord + math.rotationMatrix(lidarInAngleBodyCoord.pitch, lidarInAngleBodyCoord.roll, lidarInAngleBodyCoord.yaw).transpose() * sensorCoordVect3d;
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
	Eigen::Vector3d	bodyCoordVect3d;
	Eigen::Vector3d	mapCoordVect3d;
	Coordinate3d* mapCoord = new Coordinate3d[sizeRay];
	// ROLL, PITCH, YAW
	// 2 1 0 - YAW, PITHC, ROLL
	// Eigen::Vector3d angleEuler = orientationQuat.toRotationMatrix().eulerAngles(0, 1, 2);
	Eigen::Vector3d angleEuler = Eigen::Matrix3d(orientationQuat).eulerAngles(0, 1, 2);

	// использование tf
	tf2::Quaternion q(orientationQuat.x(), orientationQuat.y(), orientationQuat.z(), orientationQuat.w());
	// tf2::Quaternion q(-0.019, 0.037, 0.127, 0.991);
	tf2::Matrix3x3 m(q);
	double r, p, y;
	m.getRPY(r, p, y);

	// ROS_INFO("ANGLE YAW = %f\n", y * 180/M_PI);
	for	(unsigned int i = 0; i < sizeRay; i++)
	{
		convertXYZToVector3d(&bodyCoord[i], bodyCoordVect3d);
		mapCoordVect3d = currentPosition  + math.rotationMatrix(r, p, y).transpose() * bodyCoordVect3d;
		convertVector3dToXYZ(mapCoordVect3d, &mapCoord[i]);
	}

	return (mapCoord);
}

// sizeMap - размер карты, поменять в дальнешйем на три параметра(данный параметр будет зависит от дальности лидара и
			// в дальнейшем будет суммироваться, по мере движения дрона на расстояние, которое пролетел дрон) 
void			Mapping::fillOccupancyGrid(Coordinate3d& mapSize)
{
	// ParallelepipedSize numberCoordinateGrid;

	unsigned int length = round(mapSize.x/gridStep.length);
	unsigned int width = round(mapSize.y/gridStep.width);
	unsigned int height = round(mapSize.z/gridStep.height);
	// ROS_INFO("start function");
	// ROS_INFO("length = %d", length);
	// выделение памяти под координаты сетки
	// map = new Coordinate3d[length * width * height];

	// подумать о упрощении алгоритма, для заполнения сетки
	double* arrayLength = new double[length]; // координаты центра сетки в длину
	double* arrayWidth = new double[width]; // координаты центра сетки в ширину
	double* arrayHeight = new double[height]; // координаты центра сетки в высоту

	// map[0].x = origin.x + gridStep.length/2;
	// map[0].y = origin.y + gridStep.width/2;
	// map[0].z = origin.z + gridStep.height/2;
	arrayLength[0] = origin.x + gridStep.length / 2;
	arrayWidth[0] = origin.y + gridStep.width / 2;
	arrayHeight[0] = origin.z + gridStep.height / 2;
	for	(unsigned int i = 1; i < length; i++)
	{
		arrayLength[i] = arrayLength[i - 1] + gridStep.length;
	}
	for	(unsigned int i = 1; i < width; i++)
	{
		arrayWidth[i] = arrayWidth[i - 1] + gridStep.width;
	}
	for	(unsigned int i = 1; i < height; i++)
	{
		arrayHeight[i] = arrayHeight[i - 1] + gridStep.height;
	}

	for	(unsigned int count = 0, i = 0, j = 0, k = 0; count < (length * width * height); count++)
	{
		// map[count].x = arrayLength[i];
		
		// ROS_INFO("map[%d].x = %lf", i, map[count].x);
		// map[count].y = arrayWidth[j];
		// map[count].z = arrayHeight[k];
		map.x.push_back(arrayLength[i]);
		map.y.push_back(arrayWidth[j]);
		map.z.push_back(arrayHeight[k]);
		map.isOccupied.push_back(0);
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
}

// void				Mapping::updateMap()
// {
// 	if (currentPosition[0] >= updateDistance || currentPosition[0] >= updateDistance)
// 	{

// 	}

// }

// подумать о передачи параметров карты в аргумент функции
// void		Mapping::buildMap(double& x, double& y, double& z)
// {
// 	unsigned int indexX;
// 	unsigned int indexY;
// 	unsigned int indexZ;

// 	indexX = trunc(x / gridStep.length);
// }

// подумать о передачи параметров карты в аргумент функции
unsigned int		Mapping::buildMap(double& arg, double& size, double& originAxes)
{
	unsigned int index;
	index = trunc((arg / size) - originAxes);
	// ROS_INFO("INDEX = %lf", arg / size);

	return (index);
}

unsigned int	Mapping::buildMap(const double& x, const double& y, const double& z, const ParallelepipedSize& size, const Coordinate3d& originAxes)
{
	unsigned int indexX;
	unsigned int indexY;
	unsigned int indexZ;
	unsigned int index;

	indexX = trunc((x - originAxes.x) / size.length);
	indexY = trunc((y - originAxes.y) / size.width);
	indexZ = trunc((z - originAxes.z) / size.height);

	index = indexX + (indexY * (mapSize.x / size.length))
				   + (indexZ * (mapSize.x / size.length) * (mapSize.y / size.length));
	map.isOccupied[index] = 1;
	// ROS_INFO("INDEX = %lf", index);

	return (index);
}

/**
 * @brief Вывод данных с лидара в консоль
 * 
 */
void	Mapping::readDataLidar()
{
	for (unsigned int i = 0; i < laserScanValue.ranges.size(); i++)
	{
		ROS_INFO("DATA LIDAR: %f", laserScanValue.ranges[i]);
	}
}

/**
 * @brief очистка данных о массиве точек, спустя время(secClearData)
 * 
 * @param secClearData время, через которое производится очистка данных[сек]
 */
void	Mapping::clearData(double secClearData)
{
	timeSecsCurrent = ros::Time::now().toSec();
	dt = timeSecsCurrent - timeSecsPast;
	timeSecsPast = timeSecsCurrent;
	secs += dt;
	if (secs > secClearData)
	{
		points.points.clear();
		secs = 0;
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

/**
 * @brief Выделение памяти для структуры buffData
 * 
 */
void	Mapping::allocationBuffData()
{
	buffData.x = new double*[sizeRay];
	buffData.y = new double*[sizeRay];
	buffData.z = new double*[sizeRay];
	for (unsigned int k = 0; k < sizeRay; k++)
	{
		buffData.x[k] = new double[sizeBuffer];
		buffData.y[k] = new double[sizeBuffer];
		buffData.z[k] = new double[sizeBuffer];
	}
}

/**
 * @brief Ограничивает данные в диапазоне от min до max
 * 
 * @param data данные, которые требуется ограничить
 * @param min минимальное значение
 * @param max максимальное значение
 * @return значение, лежащее в диапазоне от min до max
 */
double	Mapping::saturation(const double& data, const double& min, const double& max)
{
	double result;

	if (data > max)
		result = max;
	else if (data < min)
		result = min;

	return (result);
}

/**
 * @brief Ограничивает минимальное значение данных
 * 
 * @param data данные, которые требуется ограничить
 * @param min минимальное значение
 * @return значение, которое не меньше min
 */
double	Mapping::saturationMin(const double& data, const double& min)
{
	double result;

	if (data < min)
		result = min;

	return (result);
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
bool	Mapping::checkIntervalMinMax(const double& data, const double& min, const double& max)
{
	bool result;

	result = false;
	if (data < max && data > min)
		result = true;

	return (result);
}

/**
 * @brief Визуализация карты
 * 
 */
void			Mapping::visualizationOccupancyGrid()
{
	// Coordinate3d* sensorCoord;
	// Coordinate3d* bodyCoord;
	// Coordinate3d* mapCoord;
	// clearData(10);
	// readDataLidar();
	if (countMeasurement < 3)
	{
		ParallelepipedSize numberCoordinateGrid;

		parallelepipedGrid.header.stamp = ros::Time::now();
		numberCoordinateGrid.length = round(mapSize.x/gridStep.length);
		numberCoordinateGrid.width = round(mapSize.y/gridStep.width);
		numberCoordinateGrid.height = round(mapSize.z/gridStep.height);
		ROS_INFO("p.x = %lf", p.x);
		for (unsigned int i = 0; i < numberCoordinateGrid.length * numberCoordinateGrid.width * numberCoordinateGrid.height; i++)
		{
			p.x = map.x[i];
			// ROS_INFO("p.x = %lf", p.x);
			p.y = map.y[i];
			p.z = map.z[i];
			// ROS_INFO("p.x = %lf, p.y = %lf, p.z = %lf", p.x, p.y, p.z);
			parallelepipedGrid.points.push_back(p);
		}
		// ROS_INFO("--------------------------------------------------------------------");
		marker_pub.publish(parallelepipedGrid);
		countMeasurement++;
	}
}

double		Mapping::calculateCorrelation(Map map, Map particle)
{
	double	r_AB; // коэффициент корреляции
	double	meanMapValue; // среднее значение карты(0 или 1)
	double	meanMapValueParticle; // среднее значение карты для частицы
	double	sumMapDotParticle;
	double	sumMapSquare;
	double	sumParticleSquare;

	meanMapValue = calculateMeanValueMap(map.isOccupied);
	meanMapValueParticle = calculateMeanValueMap(particle.isOccupied);
	sumMapDotParticle = 0;
	sumMapSquare = 0;
	sumParticleSquare = 0;
	for	(unsigned int i = 0; i < map.isOccupied.size(); i++)
	{
		sumMapDotParticle += (map.isOccupied[i] - meanMapValue)
						  * (particle.isOccupied[i] - meanMapValueParticle);
		sumMapSquare += math.squaring((map.isOccupied[i] - meanMapValue));
		sumParticleSquare += math.squaring((particle.isOccupied[i] - meanMapValueParticle));
	}
	r_AB = sumMapDotParticle / (sqrt(sumMapSquare * sumParticleSquare));

	return (r_AB);
}

// убрать в math.cpp
double		Mapping::calculateMeanValueMap(const std::vector<double>& mapValue)
{
	if (mapValue.size() == 0)
	{
		throw "vector size = 0";
	}
	else
	{
		double mean;
		double sumMapValue;

		sumMapValue = 0;
		for	(unsigned int i = 0; i < mapValue.size(); i++)
		{
			sumMapValue += mapValue[i];
		}
		mean = sumMapValue / (mapValue.size());

		return (mean);
	}
}

int			Mapping::saveMapInFile(char* fileName)
{
	std::ofstream file(fileName);

	file.clear();
	for	(unsigned int i = 0; i < map.isOccupied.size(); i++)
		file << map.x[i] << " " << map.y[i] << " " << map.z[i] << " " << map.isOccupied[i] << "\n";
	file.close();
}
