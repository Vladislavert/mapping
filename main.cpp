// #include "rosBridge.hpp"
#include "mapping.hpp"
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "race_main");
	ros::NodeHandle n;

	ros::Rate rate(20);

	// rosBridge bridge(n);

	ROS_INFO("Ready");
	
	Mapping	mapping(n);

	while (ros::ok())
	{
		mapping.publisherMapping();

		ros::spinOnce();
		// if (ros::Time::now().toSec() > 200)
		// {
		// 	ROS_INFO("save map");
		// 	mapping.saveMapInFile("/home/regislab/catkin_ws/src/UAVsandbox/fileMap");
		// }
		rate.sleep();
	}

}