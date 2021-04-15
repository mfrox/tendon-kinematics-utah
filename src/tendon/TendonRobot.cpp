/**
 * This code is a manual C++ conversion from Matlab code written by another
 * person
 *
 * Comments from the author:
 *
 *   The comments should give you an idea of how to use the function, but let
 *   us know if the usage is unclear.  We parameterized tendon path in terms of
 *   distance and angle from the backbone, both defined by arbitrary
 *   polynomials whose coefficients are stored in matrices C and D.  You input
 *   the routing coefficient matrices C and D along with a vector of tendon
 *   tensions tau.  An example configuration is implemented if no arguments are
 *   entered.   The program should efficiently solve the problem and integrate
 *   the full shape in about 0.1 seconds.  You can comment out the plots, of
 *   course.
 *
 * The paper first describing the kinematic model:
 *
 *   https://ieeexplore.ieee.org/abstract/document/5957337
 *
 * A recent paper that looks at stiffness (maybe)
 *
 *   https://ieeexplore.ieee.org/document/8606257
 */

#include "TendonRobot.h"

#include "BackboneSpecs.h"
#include "TendonSpecs.h"
#include "get_r_info.h"
#include "solve_initial_bending.h"
#include "tendon_deriv.h"
#include <util/macros.h>
#include <util/poly.h>
#include <util/vector_ops.h>

#include <cpptoml/toml_conversions.h>

#include <Eigen/Core>

#include <boost/numeric/odeint.hpp>

#include <algorithm>
#include <iterator>
#include <random>
#include <stdexcept>
#include <vector>

namespace E = Eigen;
namespace ode = boost::numeric::odeint;

using util::poly_at;
using util::poly_der;

namespace tendon {

namespace {

struct StiffnessMatrices {
  E::Matrix3d K_bt;
  E::Matrix3d K_se;
  E::Matrix3d K_bt_inv;
  E::Matrix3d K_se_inv;
};

/**
 * Creates and returns the stiffness matrices and their inverses
 *
 * @param specs: backbone specifications
 *
 * @return a tuple of four matrices
 *   [K_bt, K_se, K_bt_inv, K_se_inv] = get_stiffness_matrices(specs)
 * @return K_bt: Stiffness of torsion and bending
 * @return K_se: Stiffness of shear and elongation
 * @return K_bt_inv: Inverse of K_bt
 * @return K_se_inv: Inverse of K_se
 */
StiffnessMatrices get_stiffness_matrices(const BackboneSpecs &specs) {
  auto [L, dL, ro, ri, E, nu] = specs;
  UNUSED_VAR(L);
  UNUSED_VAR(dL);
  auto ro2 = ro*ro;
  auto ri2 = ri*ri;
  auto I = (1.0/4.0) * M_PI * (ro2*ro2 - ri2*ri2);
  auto Ar = M_PI * (ro2 - ri2);
  auto J = 2 * I;
  auto Gmod = E / (2 * (1 + nu));

  //std::cout
  //  << "  L:          " << L << "\n"
  //     "  ro:         " << ro << "\n"
  //     "  ri:         " << ri << "\n"
  //     "  E:          " << E << "\n"
  //     "  nu:         " << nu << "\n"
  //     "  I:          " << I << "\n"
  //     "  Ar:         " << Ar << "\n"
  //     "  J:          " << J << "\n"
  //     "  Gmod:       " << Gmod << "\n";

  E::Matrix3d K_bt = E::Matrix3d::Zero();
  K_bt(0, 0) = E * I;
  K_bt(1, 1) = E * I;
  K_bt(2, 2) = J * Gmod;

  E::Matrix3d K_bt_inv = E::Matrix3d::Zero();
  K_bt_inv(0, 0) = 1 / (E * I);
  K_bt_inv(1, 1) = 1 / (E * I);
  K_bt_inv(2, 2) = 1 / (J * Gmod);

  E::Matrix3d K_se = E::Matrix3d::Zero();
  K_se(0, 0) = Gmod * Ar;
  K_se(1, 1) = Gmod * Ar;
  K_se(2, 2) = E * Ar;

  E::Matrix3d K_se_inv = E::Matrix3d::Zero();
  K_se_inv(0, 0) = 1 / (Gmod * Ar);
  K_se_inv(1, 1) = 1 / (Gmod * Ar);
  K_se_inv(2, 2) = 1 / (E * Ar);

  return {K_bt, K_se, K_bt_inv, K_se_inv};
}

/** Integrate using Simpson's method on a sequence of equally spaced values
 *
 * Note: length of vals needs to be even.  If it is odd, we will do a trapezoid
 * for the last interval.
 *
 * @param vals: evaluations of the function to integrate
 * @param dx: constant interval of dependent variable between each function val
 *
 * @return integral value using the Simpson's method
 */
double simpsons(const std::vector<double> &vals, double dx) {
  auto N = vals.size();
  if (N < 2) { return 0.0; }

  double odd_man_out = 0.0;
  if (N % 2 != 0) { // do trapezoid for the last segment
    odd_man_out = 0.5 * dx * (vals[N-1] + vals[N]);
    N--;
  }

  double integral = vals[0] + vals[N];
  int next_multiplier = 2;
  int multiplier = 4;
  for (size_t i = 1; i < N - 1; i++) {
    integral += multiplier * vals[i];
    std::swap(multiplier, next_multiplier); // alternate coefficients
  }
  return odd_man_out + (integral * dx / 3.0);
}

} // end of unnamed namespace

std::vector<double> TendonRobot::random_state() const {
  static thread_local std::mt19937 generator;

  std::vector<double> state;

  for (auto &tendon : tendons) {
    std::uniform_real_distribution<double> dist(0.0, tendon.max_tension);
    state.push_back(dist(generator));
  }

  if (enable_rotation) {
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    state.push_back(dist(generator));
  }

  if (enable_retraction) {
    std::uniform_real_distribution<double> dist(0.0, specs.L);
    state.push_back(dist(generator));
  }

  return state;
}

/** Calculates the shape at zero tensions (home position) */
TendonResult TendonRobot::home_shape(double s_start) const {
  const double tol = 1e-10;
  if (s_start < -tol || s_start > specs.L + tol) {
    throw std::invalid_argument("s_start outside of backbone length range");
  }
  if (s_start <   0.0  ) { s_start =   0.0;   } // truncate
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  TendonResult res{};
  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(E::Vector3d{0.0, 0.0, 0.0});
    res.R.emplace_back(E::Matrix3d::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    return res;
  }

  res.t = util::range(s_start, specs.L, specs.dL);
  auto N = res.t.size();
  res.p.resize(N);
  std::transform(res.t.begin(), res.t.end(), res.p.begin(),
      [s_start](double t) { return E::Vector3d{0.0, 0.0, t - s_start}; });
  res.R = decltype(res.R)(N, E::Matrix3d::Identity());
  res.L = specs.L - s_start;

  // populate res.L_i
  for (auto &tendon : tendons) {
    // check for closed form
    if (tendon.is_straight()) {

      res.L_i.emplace_back(res.L);

    } else if (tendon.is_helix()) {

      auto &d0 = tendon.D[0];
      auto &c1 = tendon.C[1];
      double scaling = std::sqrt(1 + d0*d0*c1*c1);
      res.L_i.emplace_back(res.L * scaling);

    } else {

      // do numerical integration
      auto Cdot = poly_der(tendon.C);
      auto Ddot = poly_der(tendon.D);
      auto &D = tendon.D;
      std::vector<double> ldot_vals(res.t.size());
      std::transform(res.t.begin(), res.t.end(), ldot_vals.begin(),
          [&D, &Cdot, &Ddot](double t) {
            return std::sqrt(
                std::pow(poly_at(Ddot, t), 2)
                + std::pow(poly_at(D, t), 2) * std::pow(poly_at(Cdot, t), 2)
                + 1);
          });
      res.L_i.emplace_back(simpsons(ldot_vals, specs.dL));

    }
  }

  return res;
}

/**
 * Calculates the shape of the tendon-actuated robot given the tendon paths and
 * tensions.
 *
 * @param tau: Tensions of each tendon
 * @param s_start: starting value of s (i.e., retraction amount)
 *
 * @return TendonResult (see struct TendonResult)
 */
TendonResult TendonRobot::tension_shape(const std::vector<double> &tau,
                                        double s_start) const
{
  const double tol = 1e-10;
  std::vector<double> tau_copy = tau;
  auto N = tendons.size();
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau are not the same length");
  }
  if (std::any_of(tau.begin(), tau.end(),
                  [&tol](double tension) { return tension < -tol; }))
  {
    using util::operator<<;
    std::cerr << "Negative tensions requested: tau = " << tau << std::endl;
    throw std::invalid_argument("negative tensions are not allowed");
  }
  // truncate
  std::transform(tau_copy.begin(), tau_copy.end(), tau_copy.begin(),
                 [](double val) { return (val < 0 ? 0 : val); });

  TendonResult res{};
  if (s_start < -tol || s_start > specs.L + tol) {
    throw std::invalid_argument("s_start outside of backbone length range");
  }
  if (s_start <   0.0  ) { s_start =   0.0;   } // truncate
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(E::Vector3d{0.0, 0.0, 0.0});
    res.R.emplace_back(E::Matrix3d::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    return res;
  }

  //auto [K_bt, K_se, K_bt_inv, K_se_inv] = get_stiffness_matrices(specs);
  auto Ks = get_stiffness_matrices(specs);

  // calculate initial conditions
  E::Vector3d v_guess(0, 0, 1);
  E::Vector3d u_guess(0, 0, 0);

  //std::cout
  //  << "  K_bt:       [" << K_bt.row(0) << "]\n"
  //     "              [" << K_bt.row(1) << "]\n"
  //     "              [" << K_bt.row(2) << "]\n"
  //     "  K_se:       [" << K_se.row(0) << "]\n"
  //     "              [" << K_se.row(1) << "]\n"
  //     "              [" << K_se.row(2) << "]\n"
  //     "  K_bt_inv:   [" << K_bt_inv.row(0) << "]\n"
  //     "              [" << K_bt_inv.row(1) << "]\n"
  //     "              [" << K_bt_inv.row(2) << "]\n"
  //     "  K_se_inv:   [" << K_se_inv.row(0) << "]\n"
  //     "              [" << K_se_inv.row(1) << "]\n"
  //     "              [" << K_se_inv.row(2) << "]\n"
  //     "  v_guess:    [" << v_guess.transpose() << "]\n"
  //     "  u_guess:    [" << u_guess.transpose() << "]\n";
  auto [v0, u0, iters] = solve_initial_bending(v_guess, u_guess,
      tendons, tau_copy, Ks.K_bt_inv, Ks.K_se_inv, 30, 1e-5, 1e-6, s_start);
  UNUSED_VAR(iters);

  //std::cout << "  iterations: " << iters << "\n"
  //          << "  v0:         [" << v0.transpose() << "]\n"
  //          << "  u0:         [" << u0.transpose() << "]\n";

  State x_init(19 + N, 0.0);
  // Initial rotation is identity
  x_init[3] = 1;
  x_init[7] = 1;
  x_init[11] = 1;
  std::copy_n(v0.data(), 3, std::begin(x_init) + 12);
  std::copy_n(u0.data(), 3, std::begin(x_init) + 15);

  // solve initial value ODE
  auto deriv = [this, &tau_copy, &Ks]
               (const State &x, State &dxdt, const double t)
    { tendon_deriv(x, dxdt, t, this->tendons, tau_copy, Ks.K_bt, Ks.K_se); };
  std::vector<E::VectorXd> state_derivs;
  std::vector<E::VectorXd> states;
  std::vector<double> t;
  auto observer =
      [&state_derivs, &states, &t, &deriv](const State &x, double time) {
        State dxdt;
        deriv(x, dxdt, time);

        E::VectorXd v_x(x.size());
        E::VectorXd v_dxdt(dxdt.size());
        std::copy_n(&x[0], x.size(), v_x.data());
        std::copy_n(&dxdt[0], dxdt.size(), v_dxdt.data());

        t.emplace_back(time);
        states.emplace_back(v_x);
        state_derivs.emplace_back(v_dxdt);
      };

  ode::runge_kutta4<State> stepper;
  ode::integrate_const(stepper, deriv, x_init, s_start, specs.L,
                       std::min(specs.dL, specs.L - s_start), observer);

  

  // populate res
  res.t = std::move(t);
  res.p.resize(states.size());
  res.R.resize(states.size());
  res.L_i.resize(N);
  std::transform(states.begin(), states.end(), res.p.begin(),
      [](const E::VectorXd &x) {
        return x.head<3>();
      });
  std::transform(states.begin(), states.end(), res.R.begin(),
      [](const E::VectorXd &x) {
        return E::Matrix3d(x.data() + 3);
      });
  res.L = states.back()[18];
  std::copy_n(states.back().data() + 19, N, res.L_i.begin());

  return res;
}

bool TendonRobot::is_valid(const std::vector<double> &state,
                           const TendonResult &_home_shape,
                           const TendonResult &_shape) const
{
  auto dl = calc_dl(_home_shape.L_i, _shape.L_i);
  if (!is_within_length_limits(dl)) {
    return false;
  }
  for (size_t i = 0; i < tendons.size(); i++) {
    if (state[i] < 0.0 || tendons[i].max_tension < state[i]) {
      return false;
    }
  }
  return true;
}

cpptoml::table_ptr TendonRobot::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("tendon_robot", tbl);
  tbl->insert("radius", r);

  auto specs_tbl = specs.to_toml()->get("backbone_specs")->as_table();
  container->insert("backbone_specs", specs_tbl);

  if (!tendons.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &t : tendons) {
      tbl_array->push_back(t.to_toml());
    }
    container->insert("tendons", tbl_array);
  }

  tbl->insert("enable_rotation", enable_rotation);
  tbl->insert("enable_retraction", enable_retraction);

  return container;
}

TendonRobot TendonRobot::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto robot_tbl = tbl->get("tendon_robot")->as_table();
  if (!robot_tbl) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'tendon_robot': not a table");
  }

  auto r = robot_tbl->get("radius")->as<double>();
  if (!r) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'r': not a double");
  }
  TendonRobot robot;
  robot.r = r->get();
  robot.specs = tendon::BackboneSpecs::from_toml(tbl);

  if (tbl->contains("tendons")) {
    auto tendons = tbl->get("tendons")->as_table_array();
    if (!tendons) {
      throw cpptoml::parse_exception("Wrong type detected for 'tendons'");
    }
    for (auto &sub_tbl : tendons->get()) {
      robot.tendons.emplace_back(tendon::TendonSpecs::from_toml(sub_tbl));
    }
  }

  if (robot_tbl->contains("enable_rotation")) {
    auto enable_rotation = robot_tbl->get("enable_rotation")->as<bool>();
    if (!enable_rotation) {
      throw cpptoml::parse_exception("Wrong type detected for enable_rotation");
    }
    robot.enable_rotation = enable_rotation->get();
  }

  if (robot_tbl->contains("enable_retraction")) {
    auto enable_retraction = robot_tbl->get("enable_retraction")->as<bool>();
    if (!enable_retraction) {
      throw cpptoml::parse_exception("Wrong type detected for enable_retraction");
    }
    robot.enable_retraction = enable_retraction->get();
  }

  return robot;
}

} // end of namespace tendon
