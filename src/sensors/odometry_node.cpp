#include <boost/program_options.hpp>
#include <ros/ros.h>
#include "odisseus/odisseus_odometry.hpp"


int main(int argc, char ** argv) {

  double frequency;
  double error_vx_systematic;
  double error_vx_random;
  double error_wz_systematic;
  double error_wz_random;

  namespace po = boost::program_options;
  po::options_description description("Recognised options");
  description.add_options()
      ("help,h", "print help message")
      ("frequency,f", po::value<double>(&frequency)->default_value(1.), "set measurement frequency (Hz)")
      ("error-vx-systematic,X", po::value<double>(&error_vx_systematic)->default_value(0.), "set systematic error on X velocity")
      ("error-vx-random,x", po::value<double>(&error_vx_random)->default_value(0.), "set random error on X velocity")
      ("error-wz-systematic,T", po::value<double>(&error_wz_systematic)->default_value(0.), "set systematic error on angular velocity")
      ("error-wz-random,t", po::value<double>(&error_wz_random)->default_value(0.), "set random error on angular velocity")
      ("visualize,v", "visualize positioning system measurement");
      
  po::variables_map variables_map;
  po::store(po::parse_command_line(argc, argv, description), variables_map);
  po::notify(variables_map);

  if (variables_map.count("help")) {
    std::cout << description << std::endl;
    return EXIT_FAILURE;
  }

  // initialize ROS  
  ros::init(argc, argv, "turtle_odometry");
  ros::NodeHandle node_handle;
  
  odisseus::odo::OdometryInput odo_input;
  odo_input.frequency  =frequency;
  odo_input.error_vx_systematic = error_vx_systematic;
  odo_input.error_vx_random = error_vx_random;
  odo_input.error_wz_systematic = error_wz_systematic;
  odo_input.error_wz_random = error_wz_random;
  odo_input.visualize = variables_map.count("visualize")? true: false;

  odisseus::odo::OdisseusOdometry odisseus_odometry{node_handle, odo_input};
  odisseus_odometry.spin();

  return EXIT_SUCCESS;
}
