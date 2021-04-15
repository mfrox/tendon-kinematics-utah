#ifndef TENDON_DERIV_H
#define TENDON_DERIV_H

#include <Eigen/Core>

#include <vector>

namespace tendon {

using State = std::vector<double>;

struct TendonSpecs;

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se);

} // end of namespace tendon


#endif // TENDON_DERIV_H
