#ifndef TENDON_RESULT_H
#define TENDON_RESULT_H

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <vector>

namespace cpptoml {
class table;
}

namespace tendon {

/// Results from calculating the tendon shape
struct TendonResult {
  std::vector<double> t;           // parameterization value
  std::vector<Eigen::Vector3d> p;  // p[j] = backbone position at t[j]
  std::vector<Eigen::Matrix3d> R;  // R[j] = rotation matrix from base frame at t[j]
  double L = 0.020;                // Length of the backbone in meters
  std::vector<double> L_i;         // L[i] = total length of tendon i

  std::shared_ptr<cpptoml::table> to_toml() const;
  static TendonResult from_toml(std::shared_ptr<cpptoml::table> tbl);

  /** rotate this object about the base-frame's z-axis
   *
   * Recalculates the positions and rotations only.
   *
   * @param theta: angle in radians about the z-axis using the right-hand rule
   */
  void rotate_z(double theta);
};


} // end of namespace tendon

#endif // TENDON_RESULT_H
