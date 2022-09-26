#ifndef hiros_skeleton_visualizer_Visualizer_h
#define hiros_skeleton_visualizer_Visualizer_h

// ROS dependencies
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Custom external packages dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"
#include "skeletons/types.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
namespace skeletons {

class Visualizer : public rclcpp::Node {
 public:
  Visualizer();
  ~Visualizer();

 private:
  struct Parameters {
    std::string input_topic{};
    std::string output_topic{};

    bool publish_tfs{false};

    int seed{};
    double lifetime{-1.};
    float alpha{1.};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRosTopics();

  visualization_msgs::msg::MarkerArray getMarkerArray() const;
  void addIds(visualization_msgs::msg::MarkerArray& msg) const;
  void addMarkers(visualization_msgs::msg::MarkerArray& msg) const;
  void addLinks(visualization_msgs::msg::MarkerArray& msg) const;
  void addVelocities(visualization_msgs::msg::MarkerArray& msg) const;
  void addAccelerations(visualization_msgs::msg::MarkerArray& msg) const;

  void publishTfs();

  std_msgs::msg::ColorRGBA getColor(const int& skel_id) const;
  geometry_msgs::msg::TransformStamped ks2tf(
      const std::string& name,
      const hiros::skeletons::types::KinematicState& ks) const;

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      skel_group_sub_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  Parameters params_{};

  hiros::skeletons::types::SkeletonGroup skeleton_group_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
