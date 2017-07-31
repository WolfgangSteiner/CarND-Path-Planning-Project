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

  Trajectory(double end_s_d, double end_d, double delta_s, double delta_t);

  std::vector<Eigen::VectorXd> GetTrajectory() const;
  Eigen::VectorXd EvalAt(double t) const;

  void Finalize(const Eigen::VectorXd& aStartState, double aStartTime);
  bool IsFinished(double t) const;


public:
  static double SDerivCoeff(int idx, int deriv);
  static double SEvalAt(const Eigen::VectorXd& c, double t, int deriv);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, double t);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, const Eigen::VectorXd& d_coeffs, double t);


private:
  Eigen::VectorXd mStartState;
  Eigen::VectorXd mEndState;
  Eigen::VectorXd mSCoeffs;
  Eigen::VectorXd mDCoeffs;
  bool mIsFinalized{false};
  double mDuration{0.0};
  double mStartTime{0.0};
  double current_t_{0.0};
  double dt_{0.02};
};


//==============================================================================================
#endif //TRAJECTORY_H
//==============================================================================================
