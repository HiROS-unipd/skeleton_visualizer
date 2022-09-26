#include "skeleton_visualizer/Visualizer.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::skeletons::Visualizer>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
