// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_visualizer/Visualizer.h"

const double k_scale{0.02};

hiros::skeletons::Visualizer::Visualizer() : Node("hiros_skeleton_visualizer") {
  start();
}

hiros::skeletons::Visualizer::~Visualizer() { stop(); }

void hiros::skeletons::Visualizer::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer... RUNNING"
                                    << BASH_MSG_RESET);
}

void hiros::skeletons::Visualizer::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer... STOPPED"
                                    << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::skeletons::Visualizer::configure() {
  if (params_.publish_tfs) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  getParams();
  setupRosTopics();
}

void hiros::skeletons::Visualizer::getParams() {
  getParam("input_topic", params_.input_topic);
  getParam("output_topic", params_.output_topic);
  getParam("publish_tfs", params_.publish_tfs);
  getParam("seed", params_.seed);
  getParam("lifetime", params_.lifetime);
  getParam("alpha", params_.alpha);
}

void hiros::skeletons::Visualizer::setupRosTopics() {
  skel_group_sub_ =
      create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
          params_.input_topic, 10,
          std::bind(&Visualizer::callback, this, std::placeholders::_1));

  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      params_.output_topic, 10);
}

visualization_msgs::msg::MarkerArray
hiros::skeletons::Visualizer::getMarkerArray() const {
  if (skeleton_group_.skeletons.empty()) {
    return visualization_msgs::msg::MarkerArray();
  }

  visualization_msgs::msg::MarkerArray msg{};

  addIds(msg);
  addMarkers(msg);
  addLinks(msg);
  addVelocities(msg);
  addAccelerations(msg);

  return msg;
}

void hiros::skeletons::Visualizer::addIds(
    visualization_msgs::msg::MarkerArray& msg) const {
  visualization_msgs::msg::Marker m{};

  m.header.frame_id = skeleton_group_.frame;
  m.ns = "id";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.;
  m.lifetime = rclcpp::Duration::from_seconds(params_.lifetime);
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.scale.z = 0.2;

  for (const auto& skeleton : skeleton_group_.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = rclcpp::Time(static_cast<long>(skeleton.src_time * 1e9));
      ++m.id;
      m.text = std::to_string(skeleton.id);
      m.color = getColor(skeleton.id);

      auto centroid{skeletons::utils::centroid(skeleton)};
      if (!skeletons::utils::isNaN(centroid.pose)) {
        m.pose.position = skeletons::utils::toPointMsg(centroid.pose.position);

        msg.markers.push_back(m);
      }
    }
  }
}

void hiros::skeletons::Visualizer::addMarkers(
    visualization_msgs::msg::MarkerArray& msg) const {
  visualization_msgs::msg::Marker m{};

  m.header.frame_id = skeleton_group_.frame;
  m.ns = "markers";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.;
  m.lifetime = rclcpp::Duration::from_seconds(params_.lifetime);
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.scale.x = 2 * k_scale;
  m.scale.y = m.scale.x;
  m.scale.z = m.scale.y;

  for (const auto& skeleton : skeleton_group_.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = rclcpp::Time(static_cast<long>(skeleton.src_time * 1e9));
      ++m.id;
      m.color = getColor(skeleton.id);
      m.points.clear();

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position)) {
          m.points.push_back(
              skeletons::utils::toPointMsg(marker.center.pose.position));
        }
      }

      msg.markers.push_back(m);
    }
  }
}

void hiros::skeletons::Visualizer::addLinks(
    visualization_msgs::msg::MarkerArray& msg) const {
  visualization_msgs::msg::Marker m{};

  m.header.frame_id = skeleton_group_.frame;
  m.ns = "links";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.;
  m.lifetime = rclcpp::Duration::from_seconds(params_.lifetime);
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.scale.x = k_scale;
  m.scale.y = 2 * m.scale.x;
  m.points.resize(2);

  for (const auto& skeleton : skeleton_group_.skeletons) {
    if (!skeleton.links.empty()) {
      m.header.stamp = rclcpp::Time(static_cast<long>(skeleton.src_time * 1e9));
      m.color = getColor(skeleton.id);

      for (const auto& link : skeleton.links) {
        if (skeleton.hasMarker(link.parent_marker) &&
            skeleton.hasMarker(link.child_marker)) {
          auto parent_marker{skeleton.getMarker(link.parent_marker)};
          auto child_marker{skeleton.getMarker(link.child_marker)};

          if (!skeletons::utils::isNaN(parent_marker.center.pose.position) &&
              !skeletons::utils::isNaN(child_marker.center.pose.position)) {
            ++m.id;
            m.scale.z =
                skeletons::utils::distance(parent_marker.center.pose.position,
                                           child_marker.center.pose.position);

            m.points[0] = skeletons::utils::toPointMsg(
                parent_marker.center.pose.position);
            m.points[1] =
                skeletons::utils::toPointMsg(child_marker.center.pose.position);

            msg.markers.push_back(m);
          }
        }
      }
    }
  }
}

void hiros::skeletons::Visualizer::addVelocities(
    visualization_msgs::msg::MarkerArray& msg) const {
  visualization_msgs::msg::Marker m{};

  m.header.frame_id = skeleton_group_.frame;
  m.ns = "velocities";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.;
  m.lifetime = rclcpp::Duration::from_seconds(params_.lifetime);
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.scale.x = k_scale;
  m.scale.y = 2 * m.scale.x;
  m.scale.z = m.scale.y;
  m.points.resize(2);

  for (const auto& skeleton : skeleton_group_.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = rclcpp::Time(static_cast<long>(skeleton.src_time * 1e9));
      m.color = getColor(skeleton.id);

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position) &&
            !skeletons::utils::isNaN(marker.center.velocity.linear)) {
          ++m.id;

          m.points[0] =
              skeletons::utils::toPointMsg(marker.center.pose.position);
          m.points[1] = skeletons::utils::toPointMsg(
              marker.center.pose.position + marker.center.velocity.linear);

          msg.markers.push_back(m);
        }
      }
    }
  }
}

void hiros::skeletons::Visualizer::addAccelerations(
    visualization_msgs::msg::MarkerArray& msg) const {
  visualization_msgs::msg::Marker m{};

  m.header.frame_id = skeleton_group_.frame;
  m.ns = "accelerations";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.;
  m.lifetime = rclcpp::Duration::from_seconds(params_.lifetime);
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.scale.x = k_scale;
  m.scale.y = 2 * m.scale.x;
  m.scale.z = m.scale.y;
  m.points.resize(2);

  for (const auto& skeleton : skeleton_group_.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = rclcpp::Time(static_cast<long>(skeleton.src_time * 1e9));
      m.color = getColor(skeleton.id);

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position) &&
            !skeletons::utils::isNaN(marker.center.acceleration.linear)) {
          ++m.id;

          m.points[0] =
              skeletons::utils::toPointMsg(marker.center.pose.position);
          m.points[1] = skeletons::utils::toPointMsg(
              marker.center.pose.position + marker.center.acceleration.linear);

          msg.markers.push_back(m);
        }
      }
    }
  }
}

void hiros::skeletons::Visualizer::publishTfs() {
  if (params_.publish_tfs) {
    for (const auto& skeleton : skeleton_group_.skeletons) {
      for (const auto& link : skeleton.links) {
        if (!skeletons::utils::isNaN(link.center.pose.position) &&
            !skeletons::utils::isNaN(link.center.pose.orientation)) {
          tf_broadcaster_->sendTransform(ks2tf(
              std::to_string(skeleton.id) + "_l" + std::to_string(link.id),
              link.center));
        }
      }
    }
  }
}

std_msgs::msg::ColorRGBA hiros::skeletons::Visualizer::getColor(
    const int& skel_id) const {
  std_msgs::msg::ColorRGBA color{};
  color.a = params_.alpha;

  if (skel_id == -1) {
    color.r = 1;
    color.g = 1;
    color.b = 1;
  } else {
    std::srand(static_cast<unsigned int>(skel_id + 1));

    color.r = std::rand() / static_cast<float>(RAND_MAX);
    color.g = std::rand() / static_cast<float>(RAND_MAX);
    color.b = std::rand() / static_cast<float>(RAND_MAX);
  }

  return color;
}

geometry_msgs::msg::TransformStamped hiros::skeletons::Visualizer::ks2tf(
    const std::string& name,
    const hiros::skeletons::types::KinematicState& ks) const {
  geometry_msgs::msg::TransformStamped tf{};

  tf.header.frame_id = skeleton_group_.frame;
  tf.header.stamp = rclcpp::Time(static_cast<long>(skeleton_group_.time * 1e9));
  tf.child_frame_id = name;
  tf.transform.translation = skeletons::utils::toVector3Msg(ks.pose.position);
  tf.transform.rotation = skeletons::utils::toMsg(ks.pose.orientation);

  return tf;
}

void hiros::skeletons::Visualizer::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  skeleton_group_ = skeletons::utils::toStruct(msg);
  marker_array_pub_->publish(getMarkerArray());
  publishTfs();
}
