#ifndef ODISSEUS_ODOMETRY_HPP
#define ODISSEUS_ODOMETRY_HPP

#include <random>
#include <ros/ros.h>

namespace odisseus
{
	
	namespace odo
	{
		
		/**
		*  Helper class to handle the input to the odometry node
		*/ 
		struct OdometryInput
		{
			double frequency;
			double error_vx_systematic; 
			double error_vx_random;
			double error_wz_systematic; 
			double error_wz_random;
			bool visualize;
		};	
			
		/**
		*  Handle odometry calculations for odisseus
		*/ 
		class OdisseusOdometry {
			
			public:
			
				OdisseusOdometry(ros::NodeHandle node_handle, const OdometryInput& input);
				
				~OdisseusOdometry();
				
				void spin();
				
			private:
			
				ros::NodeHandle node_handle_;
				OdometryInput input_;
				
				ros::Subscriber turtle_pose_subscriber_;
				ros::Publisher turtle_twist_publisher_;
				
				std::default_random_engine random_generator_;
				std::normal_distribution<double> random_distribution_vx_;
				std::normal_distribution<double> random_distribution_wz_;
				
				std::string visualization_odisseus_name_;
				unsigned frame_sequence_;
				ros::Time cached_pose_timestamp_;
				odisseus::Pose cached_pose_;
				
				void odisseus_pose_callback(const odisseus::PoseConstPtr & message);
				bool is_visualization_requested() { return input_.visualize; };
				bool is_visualization_odisseus_available() { return visualization_odisseus_name_ != ""; };
				void spawn_and_configure_visualization_odisseus(const odisseus::Pose & initial_pose);
				void move_visualization_odisseus(const odisseus::Pose & measurement);
		};
		
	}
	
}


#endif
