// ROS dependencies
#include <tf2/LinearMath/Transform.h>

// Custom External Packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_visualizer/visualizer.h"

const double SCALE = 0.02;

void hiros::vis::Visualizer::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer... Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("seed", m_params.seed);
  m_nh.getParam("lifetime", m_params.lifetime);
  m_nh.getParam("alpha", m_params.alpha);
  m_nh.getParam("input_skeleton_group_topic", m_params.in_skeleton_group_topic);
  m_nh.getParam("output_marker_array_topic", m_params.out_marker_array_topic);

  m_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer... CONFIGURED" << BASH_MSG_RESET);
};

void hiros::vis::Visualizer::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer... RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void hiros::vis::Visualizer::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Visualizer...Stopping");

  if (m_skel_group_sub) {
    m_skel_group_sub.shutdown();
  }

  if (m_marker_array_pub) {
    m_marker_array_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Visualizer... STOPPED" << BASH_MSG_RESET);
  ros::shutdown();
}

void hiros::vis::Visualizer::setupRosTopics()
{
  m_skel_group_sub = m_nh.subscribe(m_params.in_skeleton_group_topic, 1, &Visualizer::callback, this);

  while (m_skel_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2,
                             "Hi-ROS Skeleton Visualizer... No input messages on input topic ' "
                               << m_params.in_skeleton_group_topic << "'");
  }

  m_marker_array_pub = m_nh.advertise<visualization_msgs::MarkerArray>(m_params.out_marker_array_topic, 1);
}

void hiros::vis::Visualizer::callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  if (!ros::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  m_skeleton_group = skeletons::utils::toStruct(t_msg);
  m_marker_array_pub.publish(getMarkerArray());
}

visualization_msgs::MarkerArray hiros::vis::Visualizer::getMarkerArray() const
{
  if (m_skeleton_group.skeletons.empty()) {
    return {};
  }

  visualization_msgs::MarkerArray out_msg;

  addIds(out_msg);
  addMarkers(out_msg);
  addLinks(out_msg);
  addBoundingBoxes(out_msg);
  addVelocities(out_msg);
  addAccelerations(out_msg);

  return out_msg;
}

void hiros::vis::Visualizer::addIds(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "id";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.scale.z = 0.2;

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = ros::Time(skeleton.src_time);
      ++m.id;
      m.text = std::to_string(skeleton.id);
      m.color = getColor(skeleton.id);

      auto centroid = skeletons::utils::centroid(skeleton);
      if (!skeletons::utils::isNaN(centroid.pose)) {
        m.pose.position = skeletons::utils::toPointMsg(centroid.pose.position);

        t_msg.markers.push_back(m);
      }
    }
  }
}

void hiros::vis::Visualizer::addMarkers(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "markers";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.scale.x = 2 * SCALE;
  m.scale.y = m.scale.x;
  m.scale.z = m.scale.y;

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = ros::Time(skeleton.src_time);
      ++m.id;
      m.color = getColor(skeleton.id);
      m.points.clear();

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position)) {
          m.points.push_back(skeletons::utils::toPointMsg(marker.center.pose.position));
        }
      }

      t_msg.markers.push_back(m);
    }
  }
}

void hiros::vis::Visualizer::addLinks(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "links";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = SCALE;
  m.scale.y = 2 * m.scale.x;
  m.points.resize(2);

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeleton.links.empty()) {
      m.header.stamp = ros::Time(skeleton.src_time);
      m.color = getColor(skeleton.id);

      for (auto& link : skeleton.links) {
        if (skeleton.hasMarker(link.parent_marker) && skeleton.hasMarker(link.child_marker)) {
          auto parent_marker = skeleton.getMarker(link.parent_marker);
          auto child_marker = skeleton.getMarker(link.child_marker);

          if (!skeletons::utils::isNaN(parent_marker.center.pose.position)
              && !skeletons::utils::isNaN(child_marker.center.pose.position)) {
            ++m.id;
            m.scale.z =
              skeletons::utils::distance(parent_marker.center.pose.position, child_marker.center.pose.position);

            m.points[0] = skeletons::utils::toPointMsg(parent_marker.center.pose.position);
            m.points[1] = skeletons::utils::toPointMsg(child_marker.center.pose.position);

            t_msg.markers.push_back(m);
          }
        }
      }
    }
  }
}

void hiros::vis::Visualizer::addBoundingBoxes(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "bounding_boxes";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  ++m.id;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = SCALE;
  m.points.resize(2);

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeletons::utils::isNaN(skeleton.bounding_box.center.pose)) {
      m.header.stamp = ros::Time(skeleton.src_time);
      ++m.id;
      m.color = getColor(skeleton.id);

      tf2::Transform box_tf_mat(skeleton.bounding_box.center.pose.orientation,
                                skeleton.bounding_box.center.pose.position);

      std::vector<geometry_msgs::Point> box_vertices(8);
      for (unsigned int i = 0; i < box_vertices.size(); ++i) {
        box_vertices[i] = skeletons::utils::toPointMsg(
          box_tf_mat
          * skeletons::types::Point(std::pow(-1, (i + 1) / 2) * (skeleton.bounding_box.height / 2),
                                    std::pow(-1, i / 2) * (skeleton.bounding_box.length / 2),
                                    std::pow(-1, i / 4) * (skeleton.bounding_box.width / 2)));
      }

      for (unsigned int i = 0; i < 4; ++i) {
        ++m.id;
        m.points[0] = box_vertices[i];
        m.points[1] = box_vertices[(i + 1) % 4];
        t_msg.markers.push_back(m);

        ++m.id;
        m.points[0] = box_vertices[4 + i];
        m.points[1] = box_vertices[4 + (i + 1) % 4];
        t_msg.markers.push_back(m);

        ++m.id;
        m.points[0] = box_vertices[i];
        m.points[1] = box_vertices[i + 4];
        t_msg.markers.push_back(m);
      }
    }
  }
}

void hiros::vis::Visualizer::addVelocities(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "velocities";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = SCALE;
  m.scale.y = 2 * m.scale.x;
  m.scale.z = m.scale.y;
  m.points.resize(2);

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = ros::Time(skeleton.src_time);
      m.color = getColor(skeleton.id);

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position)
            && !skeletons::utils::isNaN(marker.center.velocity.linear)) {
          ++m.id;

          m.points[0] = skeletons::utils::toPointMsg(marker.center.pose.position);
          m.points[1] = skeletons::utils::toPointMsg(marker.center.pose.position + marker.center.velocity.linear);

          t_msg.markers.push_back(m);
        }
      }
    }
  }
}

void hiros::vis::Visualizer::addAccelerations(visualization_msgs::MarkerArray& t_msg) const
{
  visualization_msgs::Marker m;

  m.header.frame_id = m_skeleton_group.frame;
  m.ns = "accelerations";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = ros::Duration(m_params.lifetime);
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = SCALE;
  m.scale.y = 2 * m.scale.x;
  m.scale.z = m.scale.y;
  m.points.resize(2);

  for (const auto& skeleton : m_skeleton_group.skeletons) {
    if (!skeleton.markers.empty()) {
      m.header.stamp = ros::Time(skeleton.src_time);
      m.color = getColor(skeleton.id);

      for (const auto& marker : skeleton.markers) {
        if (!skeletons::utils::isNaN(marker.center.pose.position)
            && !skeletons::utils::isNaN(marker.center.acceleration.linear)) {
          ++m.id;

          m.points[0] = skeletons::utils::toPointMsg(marker.center.pose.position);
          m.points[1] = skeletons::utils::toPointMsg(marker.center.pose.position + marker.center.acceleration.linear);

          t_msg.markers.push_back(m);
        }
      }
    }
  }
}

std_msgs::ColorRGBA hiros::vis::Visualizer::getColor(const int& t_skel_id) const
{
  std_msgs::ColorRGBA color;
  color.a = m_params.alpha;

  if (t_skel_id == -1) {
    color.r = 1;
    color.g = 1;
    color.b = 1;
  }
  else {
    std::srand(static_cast<unsigned int>(t_skel_id + 1));

    color.r = std::rand() / static_cast<float>(RAND_MAX);
    color.g = std::rand() / static_cast<float>(RAND_MAX);
    color.b = std::rand() / static_cast<float>(RAND_MAX);
  }

  return color;
}
