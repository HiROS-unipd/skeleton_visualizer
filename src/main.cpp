// ROS dependencies
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_visualizer/visualizer.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_skeleton_visualizer";
  ros::init(argc, argv, node_name);

  hiros::vis::Visualizer visualizer;
  visualizer.start();

  return 0;
}
