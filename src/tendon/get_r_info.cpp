#include "get_r_info.h"
#include "TendonSpecs.h"

#include <stdexcept>

#include <cmath>

namespace E = Eigen;

namespace {

struct PolyValues {
  E::VectorXd S;   // S[i]   = t^i
  E::VectorXd Sd;  // Sd[i]  = i * t^(i-1)
  E::VectorXd Sdd; // Sdd[i] = i * (i-1) * t^(i-2)
};

/**
 * Calculate the polynomial values for each power of the polynomial up to
 * degree N-1 along with the first and second derivatives.
 */
PolyValues get_poly_vecs(double t, size_t N) {
  // To store t^i and its dirivatives
  E::VectorXd S(N);   // S[i]   = t^i
  E::VectorXd Sd(N);  // Sd[i]  = i * t^(i-1)
  E::VectorXd Sdd(N); // Sdd[i] = i * (i-1) * t^(i-2)
  // i = 0
  S[0]   = 1;
  Sd[0]  = 0;
  Sdd[0] = 0;
  // i = 1
  if (N >= 2) {
    S[1]   = t;
    Sd[1]  = 1;
    Sdd[1] = 0;
  }
  // i >= 2
  for (size_t i = 2; i < N; i++) {
    S[i]   = t * S[i-1];
    Sd[i]  = i * S[i-1];
    Sdd[i] = i * (i-1) * S[i-2];
  }

  return {S, Sd, Sdd};
}

} // end of unnamed namespace

namespace tendon {

/**
 * Evaluates displacement (r) and first and second displacement dirivatives for
 * each tendon at location t.
 *
 * @param tendons: vector of TendonSpecs describing the shape of each tendon
 * @param t: parameter value to evaluate
 *
 * @return rInfo: displacement, first, and second derivatives for each tendon
 *    relative to the backbone at location t.
 */
rInfo get_r_info(const std::vector<TendonSpecs> &tendons, double t)
{
  if (tendons.size() == 0) {
    return rInfo{};
  }

  const size_t N_t = tendons.size();
  const size_t N_a = tendons[0].C.size();
  const size_t N_m = tendons[0].D.size();
  const size_t N_s = std::max(N_a, N_m);

  // Initialize the return value
  rInfo info;
  info.resize(N_t);

  // Get polynomial power values (and 1st + 2nd derivatives)
  auto [S, Sd, Sdd] = get_poly_vecs(t, N_s);
  E::VectorXd S_a   = S.head(N_a);
  E::VectorXd S_ad  = Sd.head(N_a);
  E::VectorXd S_add = Sdd.head(N_a);
  E::VectorXd S_m   = S.head(N_m);
  E::VectorXd S_md  = Sd.head(N_m);
  E::VectorXd S_mdd = Sdd.head(N_m);

  for (size_t j = 0; j < N_t; j++) {
    auto C_a       = tendons[j].C.dot(S_a);
    auto C_ad      = tendons[j].C.dot(S_ad);
    auto C_add     = tendons[j].C.dot(S_add);
    auto D_m       = tendons[j].D.dot(S_m);
    auto D_md      = tendons[j].D.dot(S_md);
    auto D_mdd     = tendons[j].D.dot(S_mdd);
    auto sa        = std::sin(C_a);
    auto ca        = std::cos(C_a);

    info.r[j]      = D_m   * E::Vector3d(sa, ca, 0);
    info.r_dot[j]  = D_md  * E::Vector3d(sa, ca, 0)
                   + D_m   * E::Vector3d( ca * C_ad, -sa * C_ad, 0);
    info.r_ddot[j] = D_mdd * E::Vector3d(sa, ca, 0)
                   + 2 * D_md * E::Vector3d(ca * C_ad, -sa * C_ad, 0)
                   - D_m   * E::Vector3d(sa * C_ad * C_ad, ca * C_ad * C_ad, 0)
                   + D_m   * E::Vector3d(ca * C_add, -sa * C_add, 0);
  }

  return info;
}

} // end of namespace tendon
