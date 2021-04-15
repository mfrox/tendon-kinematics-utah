#ifndef GET_R_INFO_H
#define GET_R_INFO_H

#include <Eigen/Core>
#include <vector>

namespace tendon {

struct TendonSpecs;

struct rInfo {
  std::vector<Eigen::Vector3d> r;
  std::vector<Eigen::Vector3d> r_dot;
  std::vector<Eigen::Vector3d> r_ddot;

  void resize(size_t new_size) {
    r.resize(new_size);
    r_dot.resize(new_size);
    r_ddot.resize(new_size);
  }
};

rInfo get_r_info(const std::vector<TendonSpecs> &tendons, double t);

} // end of namespace tendon

#endif // GET_R_INFO_H
