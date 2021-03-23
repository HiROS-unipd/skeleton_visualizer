#include <numeric>

// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <cv_bridge/cv_bridge.h>

// Ros Distributed Message dependencies
#include <sensor_msgs/image_encodings.h>

// Non-ROS External Dependencies
#include <opencv2/core.hpp>

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_visualizer/visualizer.h"

hiros::vis::Visualizer::Visualizer()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_ith(m_nh)
{
  m_links.push_back(std::make_pair(0, 1));
  m_links.push_back(std::make_pair(1, 2));
  m_links.push_back(std::make_pair(2, 3));
  m_links.push_back(std::make_pair(3, 4));
  m_links.push_back(std::make_pair(1, 5));
  m_links.push_back(std::make_pair(5, 6));
  m_links.push_back(std::make_pair(6, 7));
  m_links.push_back(std::make_pair(1, 8));
  m_links.push_back(std::make_pair(8, 9));
  m_links.push_back(std::make_pair(9, 10));
  m_links.push_back(std::make_pair(10, 11));
  m_links.push_back(std::make_pair(11, 22));
  m_links.push_back(std::make_pair(22, 23));
  m_links.push_back(std::make_pair(11, 24));
  m_links.push_back(std::make_pair(8, 12));
  m_links.push_back(std::make_pair(12, 13));
  m_links.push_back(std::make_pair(13, 14));
  m_links.push_back(std::make_pair(14, 19));
  m_links.push_back(std::make_pair(19, 20));
  m_links.push_back(std::make_pair(14, 21));
  m_links.push_back(std::make_pair(0, 15));
  m_links.push_back(std::make_pair(15, 17));
  m_links.push_back(std::make_pair(0, 16));
  m_links.push_back(std::make_pair(16, 18));
};

hiros::vis::Visualizer::~Visualizer(){};

void hiros::vis::Visualizer::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer...RUNNING" << BASH_MSG_RESET);
}

void hiros::vis::Visualizer::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer...Stopping");

  if (m_in_skel_sub) {
    m_in_skel_sub.shutdown();
  }

  if (m_out_img_pub) {
    m_out_img_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer...STOPPED" << BASH_MSG_RESET);
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
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer...Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("seed", m_params.seed);
  m_nh.getParam("lifetime", m_params.lifetime);
  m_nh.getParam("alpha", m_params.alpha);
  m_nh.getParam("input_skeleton_topic", m_params.in_skeleton_topic);
  m_nh.getParam("output_image_topic_name", m_params.out_image_topic_name);

  m_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer...CONFIGURED" << BASH_MSG_RESET);
};

void hiros::vis::Visualizer::skeleton_cb(skeleton_msgs::MarkerSkeletonGroupConstPtr t_msg)
{
  //  cv::Mat cv_image(540, 960, CV_8UC3, cv::Scalar(0, 0, 0));

  //  for (auto& skeleton : t_msg->skeletons) {
  //    std::srand(static_cast<unsigned int>(skeleton.id + 1));
  //    double r = std::rand() / static_cast<double>(RAND_MAX) * 255.;
  //    double g = std::rand() / static_cast<double>(RAND_MAX) * 255.;
  //    double b = std::rand() / static_cast<double>(RAND_MAX) * 255.;

  //    cv::putText(cv_image,
  //                std::to_string(skeleton.id),
  //                cv::Point(skeleton.skeleton_parts.front().keypoints.front().point.position.x,
  //                          skeleton.skeleton_parts.front().keypoints.front().point.position.y - 10),
  //                0,
  //                1.0,
  //                cv::Scalar(r, g, b));
  //    for (auto& kpg : skeleton.skeleton_parts) {
  //      for (auto& kp : kpg.keypoints) {
  //        cv::circle(cv_image, cv::Point(kp.point.position.x, kp.point.position.y), 1, cv::Scalar(r, g, b), 5);
  //      }
  //    }
  //  }

  //  cv_bridge::CvImage out_cv_img;
  //  out_cv_img.header = t_msg->header;
  //  out_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  //  out_cv_img.image = cv_image;

  //  m_out_img_pub.publish(out_cv_img.toImageMsg());

  m_rviz_pub.publish(getSkeletonMarkerArray(t_msg));
}

visualization_msgs::MarkerArray
hiros::vis::Visualizer::getSkeletonMarkerArray(skeleton_msgs::MarkerSkeletonGroupConstPtr t_msg) const
{
  visualization_msgs::MarkerArray out_msg;
  hiros::skeletons::types::MarkerSkeletonGroup mskg = hiros::skeletons::utils::toStruct(*t_msg.get());
  int id = 0;

  for (unsigned int i = 0; i < mskg.marker_skeletons.size(); ++i) {
    auto msk = mskg.marker_skeletons.at(i);

    if (!isEmpty(msk)) {
      (msk.id != -1) ? std::srand(static_cast<unsigned int>(msk.id + 1))
                     : std::srand(static_cast<unsigned int>(m_params.seed) + i + 1);
      float r = std::rand() / static_cast<float>(RAND_MAX);
      float g = std::rand() / static_cast<float>(RAND_MAX);
      float b = std::rand() / static_cast<float>(RAND_MAX);

      visualization_msgs::Marker skeleton_id;
      skeleton_id.header.frame_id = "world";
      skeleton_id.header.stamp = ros::Time::now();
      skeleton_id.ns = "pos";
      skeleton_id.action = visualization_msgs::Marker::ADD;
      skeleton_id.pose.orientation.w = 1.0;
      skeleton_id.id = id++; // skeleton_part.id = 10 * sk.id + skp.id
      skeleton_id.lifetime = ros::Duration(m_params.lifetime);
      skeleton_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      skeleton_id.text = std::to_string(msk.id);
      skeleton_id.scale.z = 0.2;
      skeleton_id.color.r = r;
      skeleton_id.color.g = g;
      skeleton_id.color.b = b;
      skeleton_id.color.a = m_params.alpha;
      if (!std::isnan(msk.marker_groups.at(0).markers.begin()->second.point.position.x)
          && !std::isnan(msk.marker_groups.at(0).markers.begin()->second.point.position.y)
          && !std::isnan(msk.marker_groups.at(0).markers.begin()->second.point.position.z)) {
        skeleton_id.pose.position.x = msk.marker_groups.at(0).markers.begin()->second.point.position.x;
        skeleton_id.pose.position.y = msk.marker_groups.at(0).markers.begin()->second.point.position.y;
        skeleton_id.pose.position.z = msk.marker_groups.at(0).markers.begin()->second.point.position.z;
      }
      out_msg.markers.push_back(skeleton_id);

      for (auto& mkg : msk.marker_groups) {
        if (mkg.first == 0) {

          visualization_msgs::Marker links;
          links.header.frame_id = "world";
          links.header.stamp = ros::Time::now();
          links.ns = "links";
          links.action = visualization_msgs::Marker::ADD;
          links.pose.orientation.w = 1.0;
          links.id = id++;
          links.lifetime = ros::Duration(m_params.lifetime);
          links.type = visualization_msgs::Marker::LINE_LIST;
          links.scale.x = 0.02;
          links.color.r = r;
          links.color.g = g;
          links.color.b = b;
          links.color.a = m_params.alpha;

          // Create the vertices for the points and lines
          for (auto& link : m_links) {
            if (hiros::skeletons::utils::hasMarker(msk, mkg.first, link.first)
                && hiros::skeletons::utils::hasMarker(msk, mkg.first, link.second)) {
              geometry_msgs::Point parent_joint, child_joint;

              parent_joint.x = msk.marker_groups.at(0).markers.at(link.first).point.position.x;
              parent_joint.y = msk.marker_groups.at(0).markers.at(link.first).point.position.y;
              parent_joint.z = msk.marker_groups.at(0).markers.at(link.first).point.position.z;

              child_joint.x = msk.marker_groups.at(0).markers.at(link.second).point.position.x;
              child_joint.y = msk.marker_groups.at(0).markers.at(link.second).point.position.y;
              child_joint.z = msk.marker_groups.at(0).markers.at(link.second).point.position.z;

              links.points.push_back(parent_joint);
              links.points.push_back(child_joint);
            }
          }

          out_msg.markers.push_back(links);
        }

        if (!mkg.second.markers.empty()) {
          visualization_msgs::Marker skeleton_part;
          skeleton_part.header.frame_id = "world";
          skeleton_part.header.stamp = ros::Time::now();
          skeleton_part.ns = "pos";
          skeleton_part.action = visualization_msgs::Marker::ADD;
          skeleton_part.pose.orientation.w = 1;
          skeleton_part.id = id++; // skeleton_part.id = 10 * sk.id + skp.id
          skeleton_part.lifetime = ros::Duration(m_params.lifetime);
          skeleton_part.type = visualization_msgs::Marker::SPHERE_LIST;
          skeleton_part.scale.x = 0.02 * static_cast<double>(m_params.alpha);
          skeleton_part.scale.y = 0.02 * static_cast<double>(m_params.alpha);
          skeleton_part.scale.z = 0.02 * static_cast<double>(m_params.alpha);
          skeleton_part.color.r = r;
          skeleton_part.color.g = g;
          skeleton_part.color.b = b;
          skeleton_part.color.a = m_params.alpha;

          for (auto& mk : mkg.second.markers) {
            if (!std::isnan(mk.second.point.position.x) && !std::isnan(mk.second.point.position.y)
                && !std::isnan(mk.second.point.position.z)) {
              geometry_msgs::Point p;
              p.x = mk.second.point.position.x;
              p.y = mk.second.point.position.y;
              p.z = mk.second.point.position.z;
              skeleton_part.points.push_back(p);

              if (!std::isnan(mk.second.point.velocity.x) && !std::isnan(mk.second.point.velocity.y)
                  && !std::isnan(mk.second.point.velocity.z)) {
                visualization_msgs::Marker velocity_marker;
                velocity_marker.header.frame_id = "world";
                velocity_marker.header.stamp = ros::Time::now();
                velocity_marker.ns = "vel";
                velocity_marker.id = id++;
                velocity_marker.type = visualization_msgs::Marker::ARROW;
                velocity_marker.action = visualization_msgs::Marker::ADD;
                velocity_marker.lifetime = ros::Duration(m_params.lifetime);
                velocity_marker.pose.orientation.w = 1;
                velocity_marker.points.push_back(p);
                geometry_msgs::Point p2;
                p2.x = p.x + mk.second.point.velocity.x * 0.1;
                p2.y = p.y + mk.second.point.velocity.y * 0.1;
                p2.z = p.z + mk.second.point.velocity.z * 0.1;
                velocity_marker.points.push_back(p2);
                velocity_marker.scale.x = 0.02;
                velocity_marker.scale.y = 0.02;
                velocity_marker.scale.z = 0.02;
                velocity_marker.color.r = r;
                velocity_marker.color.g = g;
                velocity_marker.color.b = b;
                velocity_marker.color.a = m_params.alpha;
                out_msg.markers.push_back(velocity_marker);
              }

              if (!std::isnan(mk.second.point.acceleration.x) && !std::isnan(mk.second.point.acceleration.y)
                  && !std::isnan(mk.second.point.acceleration.z)) {
                visualization_msgs::Marker acceleration_marker;
                acceleration_marker.header.frame_id = "world";
                acceleration_marker.header.stamp = ros::Time::now();
                acceleration_marker.ns = "acc";
                acceleration_marker.id = id++;
                acceleration_marker.type = visualization_msgs::Marker::ARROW;
                acceleration_marker.action = visualization_msgs::Marker::ADD;
                acceleration_marker.lifetime = ros::Duration(m_params.lifetime);
                acceleration_marker.pose.orientation.w = 1;
                acceleration_marker.points.push_back(p);
                geometry_msgs::Point p2;
                p2.x = p.x + mk.second.point.acceleration.x * 0.1;
                p2.y = p.y + mk.second.point.acceleration.y * 0.1;
                p2.z = p.z + mk.second.point.acceleration.z * 0.1;
                acceleration_marker.points.push_back(p2);
                acceleration_marker.scale.x = 0.02;
                acceleration_marker.scale.y = 0.02;
                acceleration_marker.scale.z = 0.02;
                acceleration_marker.color.r = r;
                acceleration_marker.color.g = g;
                acceleration_marker.color.b = b;
                acceleration_marker.color.a = m_params.alpha;
                out_msg.markers.push_back(acceleration_marker);
              }
            }
          }
          out_msg.markers.push_back(skeleton_part);
        }
      }
    }
  }

  return out_msg;
}

bool hiros::vis::Visualizer::isEmpty(const hiros::skeletons::types::MarkerSkeleton& t_skeleton) const
{
  for (auto& mkg : t_skeleton.marker_groups) {
    if (!mkg.second.markers.empty()) {
      return false;
    }
  }

  return true;
}
