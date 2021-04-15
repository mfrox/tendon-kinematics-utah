#ifndef TIP_CONTROL
#define TIP_CONTROL

#include <tendon/TendonRobot.h>
#include <util/macros.h>

#include <Eigen/Core>
#include <Eigen/LU>     // TODO: is this used?
#include <Eigen/Dense>  // TODO: is this used?

#include <vector>

class tip_control{

public:
  Eigen::MatrixXd Jacobian(
      tendon::TendonRobot robot,
      Eigen::Vector3d ps,
      float dist,
      const std::vector<double> &tau
      );

  std::vector<double> Dls(
      float damping,
      Eigen::Vector3d err,
      Eigen::MatrixXd J,
      float tlimit,
      const std::vector<double> &tau
      );

  std::vector<double> ClampedDls(
      float damping,
      Eigen::Vector3d err,
      Eigen::MatrixXd J,
      float tlimit,
      const std::vector<double> &tau,
      float tension_space_clamp
      );
};

//Function Definitions
inline std::vector<double> tip_control::Dls(
    float damping,
    Eigen::Vector3d err,
    Eigen::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau
    )
{
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::VectorXd tdiff;
  std::vector<double> t = tau;

  tdiff = J.transpose() * (J * J.transpose() + damping * damping * I).inverse() * err;
  for(size_t h = 0; h < tau.size(); h++) {
    t[h] += tdiff(h);
    if (t[h] > tlimit) {
      t[h] = tlimit;
    } else if (t[h] < 0) {
      t[h] = 0;
    }
  }
  return t;
}

inline std::vector<double> tip_control::ClampedDls(
    float damping,
    Eigen::Vector3d err,
    Eigen::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau,
    float tension_space_clamp)
{
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::VectorXd tdiff;
  std::vector<double> t(tau.size());

  t = tau;
  tdiff = J.transpose() * (J * J.transpose() + damping * damping * I).inverse() * err;
  if (tdiff.norm() > tension_space_clamp) {
    tdiff = tdiff.normalized() * tension_space_clamp;
  }
  for (size_t h = 0; h < tau.size(); h++) {
    t[h] += tdiff(h);
    if (t[h] > tlimit) {
      t[h] = tlimit;
    } else if (t[h] < 0) {
      t[h]=0;
    }
  }
  return t;
}


inline Eigen::MatrixXd tip_control::Jacobian(
    tendon::TendonRobot robot,
    Eigen::Vector3d ps,
    float dist,
    const std::vector<double> &tau)
{
  Eigen::MatrixXd J(3, tau.size());
  Eigen::Vector3d pos;
  std::vector<double> tau2;

  //#pragma omp parallel for
  for (size_t i = 0; i < tau.size(); i++) {
    tau2 = tau;
    tau2[i] = tau[i] + dist;
    auto [t, p, R, L, L_i] = robot.shape(tau2);
    UNUSED_VAR(L);
    pos = p.back();
    for(size_t j = 0; j < 3; j++) {
      J(j,i) = (pos[j] - ps[j]) / dist;
    }
  }
  return J;
}

#endif


