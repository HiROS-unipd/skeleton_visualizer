#ifndef hiros_skeleton_tracker_visualizer_h
#define hiros_skeleton_tracker_visualizer_h

// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>

#include "skeleton_msgs/SkeletonGroup.h"
#include "skeletons/types.h"

// OpenCV
#include "opencv2/opencv.hpp"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace vis {

    struct VisualizerParameters
    {
      std::string in_skeleton_topic = "";
      std::string out_image_topic_name = "";
    };

    class Visualizer
    {
    public:
      Visualizer();
      ~Visualizer();

      void configure();

      void start();
      void stop();

    private:
      void skeleton_cb(const skeleton_msgs::SkeletonGroupConstPtr& t_msg);

      void setupRosTopics();
      visualization_msgs::MarkerArray getSkeletonMarkerArray(const skeleton_msgs::SkeletonGroupConstPtr& t_msg) const;
      bool isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton) const;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;
      image_transport::ImageTransport m_ith;
      ros::Subscriber m_in_skel_sub;
      image_transport::Publisher m_out_img_pub;
      ros::Publisher m_rviz_pub;

      VisualizerParameters m_params{};

      bool m_configured = false;
    };
  } // namespace vis
} // namespace hiros

#endif
