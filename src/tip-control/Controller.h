#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tendon/TendonRobot.h>
#include <tip-control/tip_control.h>

#include <Eigen/Core>

#include <vector>

class Controller {
public:
  Controller(tendon::TendonRobot robot) : _robot(std::move(robot)) {}

  Controller()                                     = default; // default constructor
  Controller(const Controller &other)              = default; // copy
  Controller(Controller &&other)                   = default; // move
  Controller& operator = (const Controller &other) = default; // copy assignment
  Controller& operator = (Controller &&other)      = default; // move assignment

  tendon::TendonRobot& robot() { return _robot; }
  const tendon::TendonRobot& robot() const { return _robot; }

  void set_robot(tendon::TendonRobot robot) { _robot = std::move(robot); }

  struct ControlResult {
    std::vector<std::vector<double>> states;
    std::vector<Eigen::Vector3d> tip_positions;
    std::vector<double> errors;
    std::vector<std::vector<Eigen::Vector3d>> backbones;
    double seconds;
    bool success;
  };

  ControlResult control(const std::vector<double>& initialTau,
                        const Eigen::Vector3d& des,
                        char ver);

  struct IKResult {
    std::vector<double> state;     // solved solution robot state
    double error;                  // tip position error
    int iters;                     // number of iterations
    int num_fk_calls;              // number of forward kinematic calls
  };

  IKResult inverse_kinematics(
      const std::vector<double>& initial_state,
      const Eigen::Vector3d& des,
      int max_iters = 100,
      double mu_init = 1e-3,
      double stop_threshold_JT_err_inf = 1e-9, // |J^T err|_inf
      double stop_threshold_Dp = 1e-4,         // |Dp|_2 (like relative step size)
      double stop_threshold_err = 1e-4,        // |err|_2
      double finite_difference_delta = 1e-6,   // delta for finite difference
      bool verbose = false)
    const;

private:
  tendon::TendonRobot _robot;
};

#endif
