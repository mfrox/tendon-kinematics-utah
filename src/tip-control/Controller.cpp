#include "Controller.h"
#include <util/vector_ops.h>

#include <3rdparty/levmar-2.6/levmar.h>

#include <Eigen/LU>
#include <Eigen/Dense>

#include <chrono>

#include <cmath>

namespace E = Eigen;

namespace {

/** computes the forward kinematics of the robot end-effector
 *
 * This function is to follow the levmar function specification
 *
 * @param p: state values
 * @param x_out: output variable for the robot end-effector
 * @param p_dim: size of p
 * @param x_dim: size of x_out
 * @param robot_ptr: pointer to the tendon::TendonRobot instance
 */
void tip_forward_kinematics(double *p, double *x_out, int p_dim, int x_dim,
                            void *robot_ptr)
{
  tendon::TendonRobot *robot = static_cast<tendon::TendonRobot*>(robot_ptr);
  // other controls are allowed to be out of range, but retraction doesn't make
  // sense.  Hard-code a return value in that case.
  // TODO: put this logic in forward_kinematics() maybe?
  if (robot->enable_retraction) {
    if (p[p_dim-1] > robot->specs.L) {
      for (int i = 0; i < x_dim; ++i) {
        x_out[i] = 0.0;
      }
      return;
    }
  }
  std::vector<double> state(p, p + p_dim);
  decltype(robot->forward_kinematics(state)) backbone;
  try {
    backbone = robot->forward_kinematics(state);
  } catch (std::exception &ex) {
    using util::operator<<;
    std::cerr << "exception thrown from forward_kinematics() for state " << state;
    throw;
  }
  for (int i = 0; i < std::min(x_dim, 3); i++) {
    x_out[i] = backbone.back()[i];
  }
}

} // end of unnamed namespace

Controller::ControlResult
Controller::control(const std::vector<double>& initial_state,
                    const Eigen::Vector3d& des,
                    char ver)
{
  using secs = std::chrono::duration<double>;
  using util::operator<<;

  Controller::ControlResult results;
  results.success = false;

  std::vector<double> state = initial_state;

  int n = _robot.tendons.size();

  // TODO: use the tension and other limits specified by the robot
  E::MatrixXd J(3, n);
  int N=0;
  float step_size=100,damping=0.1,err_norm,prev_err_norm=0,dist=0.001,tol=1e-10,tlimit=200, tiptol=1e-5;
  //dist is the disturbance used in Jacobian calculation
  //tlimit is the tension limit
  //damping is the damping constant used in the LM algorithm
  //tol is the minimum error change desired after each iteration
  //tiptol is the tolerated error range from the desired position
  //err_norm is the norm of the error in the current iteration
  //rpev_err_norm is the norm of the error in the previous iteration
  float state_space_clamp = 0.01;
  tip_control tip;

  auto before = std::chrono::high_resolution_clock::now();
  while (1) {
    auto p = _robot.forward_kinematics(state);
    results.backbones.push_back(p);
    auto &ps = p.back();

    E::Vector3d err = des - ps;

    J = tip.Jacobian(_robot, ps, dist, state);
    err_norm = err.norm();

    results.tip_positions.push_back(p.back());
    results.states.push_back(state);
    results.errors.push_back(err_norm);

    if (abs(err_norm - prev_err_norm) < tol || err_norm < tiptol) {
      break;
    }

    prev_err_norm = err_norm;
    err *= step_size;
    if (ver == 'y') {
      state = tip.ClampedDls(damping, err, J, tlimit, state, state_space_clamp);
    } else if (ver == 'n') {
      state = tip.Dls(damping, err, J, tlimit, state);
    } else {
      std::cout << "\n\nNot a valid option. Use y or n\n\n";
      return results;
    }
    N += 1;
  }
  auto after = std::chrono::high_resolution_clock::now();

  results.seconds = secs(after - before).count();

  if (results.errors.back() < tiptol) {
    results.success = true;
  }

  return results;
}

/** performs inverse kinematics to a desired location of the tendon robot
 *
 * @param initial_state: initial guess at the inverse kinematics solution
 * @param des: desired end-effector position
 */
Controller::IKResult Controller::inverse_kinematics(
    const std::vector<double>& initial_state,
    const Eigen::Vector3d& des,
    int max_iters,
    double mu_init,
    double stop_threshold_JT_err_inf, // |J^T err|_inf
    double stop_threshold_Dp,         // |Dp|_2 (like relative step size)
    double stop_threshold_err,        // |err|_2
    double finite_difference_delta,   // delta for finite differences
    bool verbose)
  const
{
  using util::operator<<;

  // TODO: change to initial state
  // TODO: get limits of the rotation and retraction options

  std::vector<double> solution = initial_state;
  Eigen::Vector3d des_copy = des;

  int m = initial_state.size();
  int n = des.size();
  double info[LM_INFO_SZ];
  double workspace[LM_DIF_WORKSZ(m, n)];
  double levmar_opt[LM_OPTS_SZ];

  levmar_opt[0] = mu_init;
  levmar_opt[1] = stop_threshold_JT_err_inf;

  // looks like the thresholds are on the square norm, so square them
  levmar_opt[2] = stop_threshold_Dp * stop_threshold_Dp;
  levmar_opt[3] = stop_threshold_err * stop_threshold_err;

  levmar_opt[4] = finite_difference_delta;

  std::vector<double> lower_bounds(m, 0.0);
  std::vector<double> upper_bounds(m);
  std::transform(_robot.tendons.begin(), _robot.tendons.end(),
                 upper_bounds.begin(),
                 [](auto &tendon) { return tendon.max_tension; });
  auto n_tendons = _robot.tendons.size();
  if (_robot.enable_rotation) {
    lower_bounds[n_tendons] = -std::numeric_limits<double>::max();
    upper_bounds[n_tendons] =  std::numeric_limits<double>::max();
  }
  if (_robot.enable_retraction) {
    upper_bounds.back() = _robot.specs.L;
  }

  dlevmar_bc_dif(&tip_forward_kinematics,
                 solution.data(),
                 des_copy.data(),
                 m, n,
                 lower_bounds.data(),
                 upper_bounds.data(),
                 nullptr, // no diagonal scaling constraints
                 max_iters,
                 levmar_opt,
                 info,
                 workspace,
                 nullptr, // do not need the covariance matrix
                 static_cast<void*>(const_cast<tendon::TendonRobot*>(&_robot)));

  auto initial_tip = _robot.forward_kinematics(initial_state).back();
  auto reached_tip = _robot.forward_kinematics(solution).back();

  if (verbose) {
    std::cout << "\n"
                 "inverse_kinematics:\n"
                 "  initial state:   " << initial_state              << "\n"
                 "  initial tip:     [" << initial_tip.transpose()   << "]\n"
                 "  desired tip:     [" << des.transpose()           << "]\n"
                 "  reached tip:     [" << reached_tip.transpose()   << "]\n"
                 "  solution:        " << solution                   << "\n"
                 "  |e|_init:        " << std::sqrt(info[0])         << "\n"
                 "  |e|:             " << std::sqrt(info[1])         << "\n"
                 "  |J^T e|_inf:     " << info[2]                    << "\n"
                 "  |Dp|:            " << std::sqrt(info[3])         << "\n"
                 "  mu/max(J^T J):   " << info[4]                    << "\n"
                 "  iters:           " << info[5]                    << "\n"
                 "  term condition:  " << info[6]                    << "\n"
                 "  # fk calls:      " << info[7]                    << "\n"
                 "  # J calls:       " << info[8]                    << "\n"
                 "  # linear solves: " << info[9]                    << "\n"
                 "\n" << std::flush;
  }

  Controller::IKResult ik_soln;
  ik_soln.state        = std::move(solution);
  ik_soln.error        = std::sqrt(info[1]);
  ik_soln.iters        = info[5];
  ik_soln.num_fk_calls = info[7];

  return ik_soln;
}
