/**
 * State publisher node for Odisseus. This nodes send
 * a request about state update to the server running EKF
 * Once it receives the updated state it publishes it
 * 
 */ 


#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/tokenizer.hpp>

#include <vector>
#include <array>
#include <string>


namespace node
{
	
	const int PORT = 8001;
	const std::string URL("127.0.0.1"); 
	constexpr int STATE_SIZE = 4;
	std::array<double, STATE_SIZE> state;
	
	boost::asio::io_service io_service;
	
	void recv_state()
	{
		
		using namespace boost::asio;
		
		// connect to the server
		ip::tcp::endpoint ep( ip::address::from_string(node::URL), node::PORT);
		
		ip::tcp::socket socket(io_service);
		socket.connect(ep);
		
		boost::system::error_code error;
	
		int bytes = socket.read_some(boost::asio::buffer(state, sizeof(state)), error);
		socket.close();
		ROS_INFO("Number of bytes received: %d \n", bytes);
	
	}
	
}


int main(int argc, char **argv){

	//the name of the node this must be unique
	const std::string node_name="EKF_Node";

	//initialize ROS with the node name
	ros::init(argc, argv, node_name);
	
	ros::NodeHandle n;
	//ros::Publisher base_pub = n.advertise<sensor_msgs::JointState>("base_states", 1);
	//tf::TransformBroadcaster broadcaster;

	
	// set the frequency to send the GET request for the  state
	// to data to 10Hz
    ros::Rate loop_rate(10);
    
     while(ros::ok()){

        //std_msgs::String msg;
        //std::stringstream ss;
        //ss << " Hello ROS ";
        //msg.data = ss.str();
        
        ROS_INFO("Sending GET state request\n");
        
        node::recv_state();
        auto msg = node::state_as_string();
        
        ROS_INFO("State received %f , %f , %f , %f \n",  node::state[0], node::state[1], node::state[2], node::state[3] );
        
        // we need to publish the state
        
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
	

	return 0;
}
