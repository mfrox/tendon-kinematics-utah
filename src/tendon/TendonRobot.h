#ifndef TENDON_ROBOT_H
#define TENDON_ROBOT_H

#include <tendon/BackboneSpecs.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonSpecs.h>

#include <Eigen/Core>

#include <memory>
#include <ostream>
#include <vector>

namespace cpptoml {
class table;
}

namespace tendon {

struct TendonRobot {
  double r = 0.015;                         // radius in meters
  tendon::BackboneSpecs specs{};            // backbone specifications
  std::vector<tendon::TendonSpecs> tendons; // tendons
  bool enable_rotation    = false;          // enable rotation control
  bool enable_retraction  = false;          // enable retraction control

  size_t state_size() const {
    auto Ntau = tendons.size();
    auto Ntot = Ntau + (enable_rotation ? 1 : 0) + (enable_retraction ? 1 : 0);
    return Ntot;
  }

  std::vector<double> random_state() const;

  std::vector<Eigen::Vector3d>
  forward_kinematics(const std::vector<double> &state) const {
    auto results = this->shape(state);
    return results.p;
  }

  /** shape at the zero tension config (efficiently computed)
   *
   * Note, the s_start will be used even if retraction is disabled.
   *
   * @param s_start: starting point for shape computation.  Needs to be between
   *   0 and specs.L.  Represents control of robot insertion, i.e., only the
   *   robot from s_start to specs.L is in the workspace.
   * @return shape with zero tension
   */
  TendonResult home_shape(double s_start = 0) const;

  /** shape at zero tension
   *
   * Will pull the s_start out of the state only if retraction is enabled
   *
   * @param state: full configuration state in which retraction will be last if
   *   it is enabled.
   * @return shape with zero tension
   */
  TendonResult home_shape(const std::vector<double> &state) const {
    double retract = enable_retraction ? state.back() : 0.0;
    return home_shape(retract);
  }

  /** computes shape from state
   *
   * @param state: tendon tensions, rotation (if enabled), then retraction (if
   *     enabled)
   *
   * @return shape at the given state
   */
  TendonResult shape(const std::vector<double> &state) const {
    auto Ntau = tendons.size();
    if (state.size() != state_size()) {
      throw std::invalid_argument("State is not the right size");
    }
    std::vector<double> tau(state.begin(), state.begin() + Ntau);
    double rotate = enable_rotation ? state[Ntau] : 0.0;
    double retract = enable_retraction ? state.back() : 0.0;
    return shape(tau, rotate, retract);
  }

  /** Computes the robot shape
   *
   * @param tau: tendon tensions controls
   * @param rotation: rotation control (only used if enable_rotation)
   * @param retraction: retraction control (only used if enable_retraction)
   */
  TendonResult shape(const std::vector<double> &tau, double rotation,
                     double retraction) const
  {
    double retract = enable_retraction ? retraction : 0.0;
    auto result = tension_shape(tau, retract);
    if (enable_rotation) {
      result.rotate_z(rotation);
    }
    return result;
  }

  bool is_valid(const std::vector<double> &state,
                const TendonResult &_home_shape) const
  { return is_valid(state, _home_shape, shape(state)); }

  bool is_valid(const std::vector<double> &state,
                const TendonResult &_home_shape,
                const TendonResult &_shape) const;

  std::vector<double> calc_dl(const std::vector<double> &home_l,
                              const std::vector<double> &other_l) const
  {
    if (home_l.size() != other_l.size()) {
      throw std::out_of_range("vector size mismatch");
    }
    std::vector<double> dl(home_l.size());
    for (size_t i = 0; i < home_l.size(); i++) {
      // string extension means other is shorter
      dl[i] = home_l[i] - other_l[i];
    }
    return dl;
  }

  bool is_within_length_limits(const std::vector<double> &home_l,
                               const std::vector<double> &other_l) const
  {
    auto dl = calc_dl(home_l, other_l);
    return is_within_length_limits(dl);
  }

  bool is_within_length_limits(const std::vector<double> &dl) const {
    if (dl.size() != tendons.size()) {
      throw std::out_of_range("length mismatch");
    }
    for (size_t i = 0; i < dl.size(); i++) {
      if (dl[i] < tendons[i].min_length || tendons[i].max_length < dl[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator==(const TendonRobot &other) const {
    return r                 == other.r
        && specs             == other.specs
        && tendons           == other.tendons
        && enable_rotation   == other.enable_rotation
        && enable_retraction == other.enable_retraction;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static TendonRobot from_toml(std::shared_ptr<cpptoml::table> tbl);

private:
  /** shape at the given tendon tensions
   *
   * @param tau: tendon tensions, to be between tendon limits, but not checked.
   * @param s_start: starting point for shape computation.  Needs to be between
   *   0 and specs.L.  Represents control of robot insertion, i.e., only the
   *   robot from s_start to specs.L is in the workspace.
   * @return shape at tau starting at s_start
   */
  TendonResult tension_shape(const std::vector<double> &tau, double s_start) const;
};

inline std::ostream& operator<<(std::ostream &out, const TendonRobot &robot) {
  out << "TendonRobot{r=" << robot.r << ", specs=" << robot.specs << ", "
                      "tendons=[";
  bool first = true;
  for (auto &tendon : robot.tendons) {
    if (!first) { out << ", "; }
    out << tendon;
    first = false;
  }
  out << "], "
         "enable_rotation=" << (robot.enable_rotation ? "true" : "false")
      << ", enable_retraction=" << (robot.enable_retraction ? "true" : "false")
      << "}";
  return out;
}

} // end of namespace tendon

#endif // TENDON_ROBOT_H
