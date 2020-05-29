// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <cv_bridge/cv_bridge.h>

// Ros Distributed Message dependencies
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

// Non-ROS External Dependencies
#include <opencv2/core.hpp>

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker_visualizer/visualizer.h"

hiros::vis::Visualizer::Visualizer()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_ith(m_nh){};

hiros::vis::Visualizer::~Visualizer(){};

void hiros::vis::Visualizer::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker Visualizer...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker Visualizer...RUNNING" << BASH_MSG_RESET);
}

void hiros::vis::Visualizer::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker Visualizer...Stopping");

  if (m_in_skel_sub) {
    m_in_skel_sub.shutdown();
  }

  if (m_out_img_pub) {
    m_out_img_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker Visualizer...STOPPED" << BASH_MSG_RESET);
}

void hiros::vis::Visualizer::setupRosTopics()
{
  // Subscribe to input topics
  m_in_skel_sub = m_nh.subscribe(m_params.in_skeleton_topic, 1, &Visualizer::skeleton_cb, this);

  // Sanity check on input skeleton topic
  while (m_in_skel_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages");
  }

  m_out_img_pub = m_ith.advertise(m_params.out_image_topic_name, 1);
  m_rviz_pub = m_nh.advertise<visualization_msgs::MarkerArray>("skel_vis", 1);
}

void hiros::vis::Visualizer::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker Visualizer...Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("input_skeleton_topic", m_params.in_skeleton_topic);
  m_nh.getParam("output_image_topic_name", m_params.out_image_topic_name);

  m_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker Visualizer...CONFIGURED" << BASH_MSG_RESET);
};

void hiros::vis::Visualizer::skeleton_cb(const skeleton_msgs::SkeletonGroupConstPtr& t_msg)
{
  cv::Mat cv_image(540, 960, CV_8UC3, cv::Scalar(0, 0, 0));

  for (auto& skeleton : t_msg->skeletons) {
    std::srand(static_cast<unsigned int>(skeleton.id + 1));
    double r = std::rand() / static_cast<double>(RAND_MAX) * 255.;
    double g = std::rand() / static_cast<double>(RAND_MAX) * 255.;
    double b = std::rand() / static_cast<double>(RAND_MAX) * 255.;

    cv::putText(cv_image,
                std::to_string(skeleton.id),
                cv::Point(skeleton.skeleton_parts.front().keypoints.front().point.position.x,
                          skeleton.skeleton_parts.front().keypoints.front().point.position.y - 10),
                0,
                1.0,
                cv::Scalar(r, g, b));
    for (auto& kpg : skeleton.skeleton_parts) {
      for (auto& kp : kpg.keypoints) {
        cv::circle(cv_image, cv::Point(kp.point.position.x, kp.point.position.y), 1, cv::Scalar(r, g, b), 5);
      }
    }
  }

  cv_bridge::CvImage out_cv_img;
  out_cv_img.header = t_msg->header;
  out_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  out_cv_img.image = cv_image;

  m_out_img_pub.publish(out_cv_img.toImageMsg());
  m_rviz_pub.publish(getSkeletonMarkerArray(t_msg));
}

visualization_msgs::MarkerArray
hiros::vis::Visualizer::getSkeletonMarkerArray(const skeleton_msgs::SkeletonGroupConstPtr& t_msg) const
{
  visualization_msgs::MarkerArray out_msg;
  hiros::skeletons::types::SkeletonGroup skg = hiros::skeletons::utils::toStruct(*t_msg.get());
  int id = 0;

  for (auto& sk : skg.skeletons) {
    if (!isEmpty(sk)) {
      std::srand(static_cast<unsigned int>(sk.id + 1));
      float r = std::rand() / static_cast<float>(RAND_MAX);
      float g = std::rand() / static_cast<float>(RAND_MAX);
      float b = std::rand() / static_cast<float>(RAND_MAX);

      visualization_msgs::Marker skeleton_id;
      skeleton_id.header.frame_id = "world";
      skeleton_id.header.stamp = ros::Time::now();
      skeleton_id.ns = "skeletons";
      skeleton_id.action = visualization_msgs::Marker::ADD;
      skeleton_id.pose.orientation.w = 1.0;
      skeleton_id.id = id++; // skeleton_part.id = 10 * sk.id + skp.id
      skeleton_id.lifetime = ros::Duration(0.1);
      skeleton_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      skeleton_id.text = std::to_string(sk.id);
      skeleton_id.scale.x = 0.2;
      skeleton_id.scale.y = 0.2;
      skeleton_id.scale.z = 0.2;
      skeleton_id.color.r = r;
      skeleton_id.color.g = g;
      skeleton_id.color.b = b;
      skeleton_id.color.a = 1.0;
      skeleton_id.pose.position.x = sk.skeleton_parts.front().keypoints.front().point.position.x;
      skeleton_id.pose.position.y = sk.skeleton_parts.front().keypoints.front().point.position.y;
      skeleton_id.pose.position.z = sk.skeleton_parts.front().keypoints.front().point.position.z;
      out_msg.markers.push_back(skeleton_id);

      for (auto& skp : sk.skeleton_parts) {
        if (!skp.keypoints.empty()) {
          visualization_msgs::Marker skeleton_part;
          skeleton_part.header.frame_id = "world";
          skeleton_part.header.stamp = ros::Time::now();
          skeleton_part.ns = "skeletons";
          skeleton_part.action = visualization_msgs::Marker::ADD;
          skeleton_part.pose.orientation.w = 1.0;
          skeleton_part.id = id++; // skeleton_part.id = 10 * sk.id + skp.id
          skeleton_part.lifetime = ros::Duration(0.1);
          skeleton_part.type = visualization_msgs::Marker::SPHERE_LIST;
          skeleton_part.scale.x = 0.02;
          skeleton_part.scale.y = 0.02;
          skeleton_part.scale.z = 0.02;
          skeleton_part.color.r = r;
          skeleton_part.color.g = g;
          skeleton_part.color.b = b;
          skeleton_part.color.a = 1.0;

          for (auto& kp : skp.keypoints) {
            geometry_msgs::Point p;
            p.x = kp.point.position.x;
            p.y = kp.point.position.y;
            p.z = kp.point.position.z;
            skeleton_part.points.push_back(p);
          }

          out_msg.markers.push_back(skeleton_part);
        }
      }
    }
  }

  return out_msg;
}

bool hiros::vis::Visualizer::isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton) const
{
  bool is_empty = true;

  for (auto& kpg : t_skeleton.skeleton_parts) {
    if (!kpg.keypoints.empty()) {
      is_empty = false;
      break;
    }
  }

  return is_empty;
}
