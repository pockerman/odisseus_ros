

#include "ros/ros.h"
#include <boost/algorithm/string.hpp> //for boost::split
#include <vector>
#include <string>


int main(int argc, char **argv){

	//the name of the node this must be unique
	const std::string node_name="Odisseus_Node";

	//initialize ROS with the node name
	ros::init(argc, argv, node_name);
	
	
	return 0;
}
