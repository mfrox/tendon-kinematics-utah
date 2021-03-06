#ifndef MARKER_ARRAY_CONVERSIONS_H
#define MARKER_ARRAY_CONVERSIONS_H

#include <tendon/TendonRobot.h>
#include <tendon/get_r_info.h>
#include <vistendon/shapes.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>

#include <vector>

namespace vistendon {

namespace E = Eigen;
using Mat = E::Matrix3d;
using Quaternion = E::Quaterniond;
using Vec = E::Vector3d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct Color {
  float r = 0.0f; // red
  float g = 0.0f; // green
  float b = 0.0f; // blue
  float a = 1.0f; // alpha
};

inline Quaternion find_quat_from_zaxis(const Vec &A) {
  return Quaternion::FromTwoVectors(Vec{0, 0, 1}, A);
}

/// Populates header, ns, action, color, and id = 1
inline void populate_marker_attributes(
    Marker &marker,
    const std::string &ns = "tendon",
    const std::string &frame_id = "/map",
    const Color &color = Color{0, 1.0f, 0, 1.0f})
{
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = ns;
  marker.action = Marker::ADD;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.id = 1;
}

/// Populates type, scale, and pose of the marker
inline void populate_marker(Marker &marker, const Vec &point) {
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.pose.position.x = point[0];
  marker.pose.position.y = point[1];
  marker.pose.position.z = point[2];
  std::cerr << "Warning: visualization of points is not supported or suggested\n";
}

/// Populates type, scale, and pose of the marker
inline void populate_marker(Marker &marker, const Sphere &sphere) {
  marker.type = Marker::SPHERE;
  marker.scale.x = 2 * sphere.r; // diameter in x
  marker.scale.y = 2 * sphere.r; // diameter in y
  marker.scale.z = 2 * sphere.r; // diameter in z
  marker.pose.position.x = sphere.c[0];
  marker.pose.position.y = sphere.c[1];
  marker.pose.position.z = sphere.c[2];
}

/// Make a cylinder marker to approximate a capsule
inline void populate_marker(Marker &marker, const Capsule &capsule) {
  marker.type = Marker::CYLINDER;
  marker.pose.position.x = capsule.a[0];
  marker.pose.position.y = capsule.a[1];
  marker.pose.position.z = capsule.a[2];
  marker.scale.x = 2 * capsule.r; // diameter in x
  marker.scale.y = 2 * capsule.r; // diameter in y
  marker.scale.z = (capsule.b - capsule.a).norm();

  auto quat = find_quat_from_zaxis(capsule.b - capsule.a);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
}

template <typename Shape>
inline Marker to_marker(
    const Shape &shape,
    const std::string &ns = "environment",
    const std::string &frame_id = "/map",
    const Color &color = Color{0, 1.0f, 0, 1.0f})
{
  Marker marker;
  populate_marker_attributes(marker, ns, frame_id, color);
  populate_marker(marker, shape);
  return marker;
}

inline Marker capsule_sequence_to_line_strip(
    const TendonRobotShape &caps,
    const std::string &ns = "tendon",
    const std::string &frame_id = "/map",
    const Color &color = Color{0, 1.0f, 0, 1.0f})
{
  Marker marker;
  populate_marker_attributes(marker, ns, frame_id, color);
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = caps.r;
  for (auto &point : caps.points) {
    geometry_msgs::msg::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    marker.points.emplace_back(std::move(p));
  }
  return marker;
}

inline MarkerArray capsule_sequence_to_markers(
    const TendonRobotShape &caps,
    const std::string &ns = "tendon",
    const std::string &frame_id = "/map",
    const Color &color = Color{0, 1.0f, 0, 1.0f})
{
  MarkerArray robot;

  Marker segment;
  populate_marker_attributes(segment, ns, frame_id, color);

  // Populate marker array
  for (size_t i = 0; i < caps.size(); i++) {
    populate_marker(segment, caps[i]);
    segment.id = robot.markers.size();
    robot.markers.push_back(segment);
  }

  // put a sphere on the end.
  auto end = caps[caps.size() - 1];
  populate_marker(segment, Sphere{end.b, end.r});
  segment.id = robot.markers.size();
  robot.markers.push_back(segment);
  return robot;
}

/** Calculates the robot and tendon shapes
 *
 * Using the given shape data and the tendon routings from the robot, convert
 * the shape data into a backbone shape, and calculate the shape of the tendons
 * with the relative routing relative to the backbone shape.
 *
 * The home shape is used to calculate the correct tendon lengths for
 * visualizing how much each string is extended or retracted from the home
 * position.
 *
 * Returns a pair: (backbone shape, list(tendon shape))
 */
inline std::pair<TendonRobotShape, std::vector<TendonRobotShape>>
robot_and_tendon_shape(
    const tendon::TendonRobot &robot,
    const tendon::TendonResult &shape_data,
    const tendon::TendonResult &home_shape)
{
  TendonRobotShape backbone;
  backbone.points = shape_data.p;
  backbone.r = robot.r;

  // calculate tendon shapes
  auto tendon_r = robot.r / 20.0;
  std::vector<TendonRobotShape> tendons(robot.tendons.size(), {{}, tendon_r});
  for (size_t i = 0; i < shape_data.t.size(); i++) {
    auto rinfo = get_r_info(robot.tendons, shape_data.t[i]);
    for (size_t tendon_i = 0; tendon_i < robot.tendons.size(); tendon_i++) {
      tendons[tendon_i].points.emplace_back(
          shape_data.p[i] + shape_data.R[i] * rinfo.r[tendon_i]);
    }
  }

  // make tendon shape length match home shape length
  for (size_t i = 0; i < shape_data.L_i.size(); i++) {
    auto dL = home_shape.L_i[i] - shape_data.L_i[i];  // delta length
    auto &tpoints = tendons[i].points;
    if (dL == 0.0) {
      continue;
    } else if (dL > 0.0) {
      tpoints.emplace(tpoints.begin(), tpoints[0] - Vec{0, 0, dL});
    } else {
      dL = -dL;
      while (dL > 0.0) {
        Eigen::Vector3d link = tpoints[1] - tpoints[0];
        auto length = link.norm();
        if (length <= dL) {
          // remove the front link
          dL -= length;
          tpoints.erase(tpoints.begin());
        } else {
          // shorten the front link
          tpoints[0] = tpoints[0] + link * dL / length;
          break;
        }
      }
    }
  }

  return {backbone, tendons};
}

/** Calculates the robot and tendon shapes
 *
 * This version is a convenience that calculates the robot shape at tau but
 * uses the given home position shape.
 *
 * Returns a pair: (backbone shape, list(tendon shape))
 */
inline std::pair<TendonRobotShape, std::vector<TendonRobotShape>>
robot_and_tendon_shape(
    const tendon::TendonRobot &robot,
    const std::vector<double> &state,
    const tendon::TendonResult &home_shape)
{
  auto shape_data = robot.shape(state);
  return robot_and_tendon_shape(robot, shape_data, home_shape);
}

/** Calculates the robot and tendon shapes
 *
 * This version is a convenience that calculates the robot shape at tau and at
 * the home position shape.
 *
 * Returns a pair: (backbone shape, list(tendon shape))
 */
inline std::pair<TendonRobotShape, std::vector<TendonRobotShape>>
robot_and_tendon_shape(const tendon::TendonRobot &robot,
                       const std::vector<double> &state)
{
  auto home_shape = robot.home_shape(state);
  auto shape = robot.shape(state);
  return robot_and_tendon_shape(robot, shape, home_shape);
}

} // end of namespace vistendon

#endif // MARKER_ARRAY_CONVERSIONS_H
