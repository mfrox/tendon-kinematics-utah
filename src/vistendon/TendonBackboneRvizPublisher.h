#ifndef TENDON_BACKBONE_RVIZ_PUBLISHER_H
#define TENDON_BACKBONE_RVIZ_PUBLISHER_H

#include "vistendon/marker_array_conversions.h"
#include "vistendon/RvizMarkerArrayPublisher.h"
#include "vistendon/shapes.h"

#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <limits>
#include <string>

#include <cmath>

namespace vistendon {

class TendonBackboneRvizPublisher : public RvizMarkerArrayPublisher {
public:
  TendonBackboneRvizPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "tendon-backbone")
    : RvizMarkerArrayPublisher(node, frame, rviz_namespace)
  {
    RCLCPP_DEBUG(_logger, "Created TendonBackboneRvizPublisher");
  }

  // Note: not thread-safe
  void set_robot(const TendonRobotShape &shape) {
    _markers = capsule_sequence_to_markers(
        shape, _rviz_namespace, _frame, _color);
    RCLCPP_DEBUG(_logger, "Updated tendon backbone");
    publish();
  }

  // Note: not thread-safe
  void set_robot(const std::vector<Eigen::Vector3d> &shape, double radius) {
    set_robot(TendonRobotShape{shape, radius});
  }
};

} // end of namespace vistendon

#endif // TENDON_BACKBONE_RVIZ_PUBLISHER_H
