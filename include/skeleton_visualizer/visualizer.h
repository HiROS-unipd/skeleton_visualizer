#ifndef hiros_skeleton_visualizer_visualizer_h
#define hiros_skeleton_visualizer_visualizer_h

// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <visualization_msgs/MarkerArray.h>

// Custom External Packages dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"
#include "skeletons/types.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace vis {

    struct VisualizerParameters
    {
      int seed{};
      double lifetime{-1};
      float alpha{1};
      std::string in_skeleton_group_topic{};
      std::string out_marker_array_topic{};
    };

    class Visualizer
    {
    public:
      Visualizer() {}
      ~Visualizer() {}

      void configure();

      void start();
      void stop();

    private:
      void callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      void setupRosTopics();
      visualization_msgs::MarkerArray getMarkerArray() const;

      void addIds(visualization_msgs::MarkerArray& t_msg) const;
      void addMarkers(visualization_msgs::MarkerArray& t_msg) const;
      void addLinks(visualization_msgs::MarkerArray& t_msg) const;
      void addBoundingBoxes(visualization_msgs::MarkerArray& t_msg) const;
      void addVelocities(visualization_msgs::MarkerArray& t_msg) const;
      void addAccelerations(visualization_msgs::MarkerArray& t_msg) const;

      std_msgs::ColorRGBA getColor(const int& t_skel_id) const;

      ros::NodeHandle m_nh{"~"};

      ros::Subscriber m_skel_group_sub;
      ros::Publisher m_marker_array_pub;

      VisualizerParameters m_params{};
      hiros::skeletons::types::SkeletonGroup m_skeleton_group{};

      bool m_configured = false;
    };
  } // namespace vis
} // namespace hiros

#endif
