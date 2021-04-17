#ifndef VISTENDON_SHAPES_H
#define VISTENDON_SHAPES_H

#include <tendon/TendonResult.h>

#include <Eigen/Core>

#include <algorithm>
#include <vector>

namespace vistendon {

struct Sphere {
  Eigen::Vector3d c; // center
  double r;          // radius
};

struct Capsule {
  Eigen::Vector3d a; // first point
  Eigen::Vector3d b; // second point
  double r;          // radius
};

struct TendonRobotShape {
  std::vector<Eigen::Vector3d> points;  // points (in sequence)
  double r;                             // radius

  size_t size() const { return std::max(size_t(1), points.size()) - 1; }
  Capsule operator[](size_t i) const { return Capsule{points[i], points[i+1], r}; }

  static TendonRobotShape from_result(const tendon::TendonResult &result,
                                      double radius)
  {
    TendonRobotShape shape;
    shape.points = result.p;
    shape.r      = radius;
    return shape;
  }
};

}

#endif // VISTENDON_SHAPES_H
