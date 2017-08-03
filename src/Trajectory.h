//==============================================================================================
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
//==============================================================================================

//==============================================================================================

class TTrajectory
{
public:
  using TTrajectoryPtr = std::shared_ptr<TTrajectory>;

public:
  TTrajectory();

  TTrajectory(
    const Eigen::VectorXd& start_state,
    const Eigen::VectorXd& end_state,
    const double start_t,
    const double duration);

  TTrajectory(double end_s_d, double end_d, double delta_s, double delta_t);

  static TTrajectoryPtr SConstantVelocityTrajectory(
    double aCurrentS, double aCurrentD, double aVelocity, double aCurrentTime, double aDuration);

  static TTrajectoryPtr SVelocityKeepingTrajectory(
    const Eigen::VectorXd& aStartState, double aTargetVelocity, double aCurrentTime, double aDuration);


  std::vector<Eigen::VectorXd> GetTrajectory() const;
  Eigen::VectorXd EvalAt(double t) const;

  void Finalize(const Eigen::VectorXd& aStartState, double aStartTime);
  bool IsFinished(double t) const;

  std::tuple<double,double> MinDistanceToTrajectory(const TTrajectoryPtr apOtherTrajectory) const;

  double JerkCost() const;
  double SafetyDistanceCost(const TTrajectoryPtr apOtherTrajectory) const;

  void AddCost(double c);
  double Cost() const;

  double MinVelocity() const;
  double MaxVelocity() const;

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
  double mTimeStep{0.02};
  double mCost{0.0};
};

//==============================================================================================
#endif //TRAJECTORY_H
//==============================================================================================
