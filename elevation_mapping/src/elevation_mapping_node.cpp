/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "elevation_mapping");

  // Create node handle which is passed
  // as a parameter to the ElevationMapping class
  ros::NodeHandle nodeHandle("~");

  // Elevation mapping class object is created
  elevation_mapping::ElevationMapping elevationMap(nodeHandle);

  // Async spinner whose threads can be defined as a parameter
  ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads

  // Async spinner is started
  spinner.start();
  
  ros::waitForShutdown();
  return 0;
}
