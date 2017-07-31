//==============================================================================================
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
//==============================================================================================

class Trajectory
{
public:
  Trajectory(
    const Eigen::VectorXd& start_state,
    const Eigen::VectorXd& end_state,
    const double start_t,
    const double duration);

  std::vector<Eigen::VectorXd> GetTrajectory() const;
  Eigen::VectorXd EvalAt(double t) const;


public:
  static double SDerivCoeff(int idx, int deriv);
  static double SEvalAt(const Eigen::VectorXd& c, double t, int deriv);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, double t);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, const Eigen::VectorXd& d_coeffs, double t);


private:
  Eigen::VectorXd s_coeffs_;
  Eigen::VectorXd d_coeffs_;
  double duration_{0.0};
  double start_t_{0.0};
  double current_t_{0.0};
  double dt_{0.02};
};


//==============================================================================================
#endif //TRAJECTORY_H
//==============================================================================================
