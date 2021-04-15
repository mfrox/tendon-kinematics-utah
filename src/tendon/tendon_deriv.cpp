#include "tendon_deriv.h"
#include "get_r_info.h"
#include "TendonSpecs.h"
#include <util/vector_ops.h>

#include <Eigen/SVD>

#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace E = Eigen;

using util::hat;

namespace tendon {

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const E::Matrix3d &K_bt,
    const E::Matrix3d &K_se)
{
  auto N_t = tendons.size();
  using V3 = E::Vector3d;
  using M3 = E::Matrix3d;

  // argument validation
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau need to be the same size");
  }
  if (x.size() != 19 + N_t) {
    throw std::out_of_range("x State size is not correct");
  }
  if (dxdt.size() != 19 + N_t) {
    //throw std::out_of_range("dxdt State size is not correct");
    dxdt.resize(19 + N_t);
  }

  // Unpack the state
  const double *data = &x[0];
  V3 p(data);
  M3 R(data + 3);
  V3 v(data + 12);
  V3 u(data + 15);

  M3 vhat = hat(v);
  M3 uhat = hat(u);
  auto [r, r_dot, r_ddot] = get_r_info(tendons, t);

  // Initialize used variables
  M3 A, B, G, H;
  A = B = G = H = M3::Zero();
  V3 a, b;
  a = b = V3::Zero();
  E::VectorXd si_dot = E::VectorXd::Zero(N_t);

  for (decltype(N_t) j = 0; j < N_t; j++) {
    M3 rhat         = hat(r[j]);
    V3 pi_dot_b     = uhat * r[j] + r_dot[j] + v;
    M3 pi_dot_b_hat = hat(pi_dot_b);
    si_dot[j]       = pi_dot_b.norm();

    M3 Ai = -tau[j] * pi_dot_b_hat * pi_dot_b_hat
              / (si_dot[j] * si_dot[j] * si_dot[j]);
    M3 Bi = rhat * Ai;
    M3 Gi = -Ai * rhat;
    M3 Hi = -Bi * rhat;

    V3 ai = Ai * (uhat * pi_dot_b + uhat * r_dot[j] + r_ddot[j]);
    V3 bi = rhat * ai;

    A += Ai;
    B += Bi;
    G += Gi;
    H += Hi;
    a += ai;
    b += bi;
  }

  V3 v_minus_vstar = v - V3(0, 0, 1);
  V3 c = -uhat * K_bt * u - vhat * K_se * v_minus_vstar - b;
  V3 d = -uhat * K_se * v_minus_vstar - a;

  E::Matrix<double, 6, 6> M;
  M.block<3, 3>(0, 0) = K_se + A;
  M.block<3, 3>(0, 3) = G;
  M.block<3, 3>(3, 0) = B;
  M.block<3, 3>(3, 3) = K_bt + H;

  // solve: xi_dot = M \ [d;c]
  E::Matrix<double, 6, 1> xi;
  xi.head<3>() = d;
  xi.tail<3>() = c;
  E::Matrix<double, 6, 1> xi_dot =
    //M.bdcSvd(E::ComputeFullU | E::ComputeFullV).solve(xi);
    //M.bdcSvd(E::ComputeThinU | E::ComputeThinV).solve(xi);
    M.jacobiSvd(E::ComputeFullU | E::ComputeFullV).solve(xi);
    //M.jacobiSvd(E::ComputeThinU | E::ComputeThinV).solve(xi);

  V3 p_dot = R * v;
  M3 R_dot = R * uhat;
  V3 v_dot = xi_dot.head<3>();
  V3 u_dot = xi_dot.tail<3>();

  // store the solution
  std::copy_n(p_dot.data(), 3, std::begin(dxdt));
  std::copy_n(R_dot.data(), 9, std::begin(dxdt) + 3);
  std::copy_n(v_dot.data(), 3, std::begin(dxdt) + 12);
  std::copy_n(u_dot.data(), 3, std::begin(dxdt) + 15);
  dxdt[18] = v.norm();
  std::copy_n(si_dot.data(), N_t, std::begin(dxdt) + 19);
}

} // end of namespace tendon
