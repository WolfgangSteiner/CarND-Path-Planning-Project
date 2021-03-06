//==============================================================================================
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include <memory>
//==============================================================================================
#define MCostProperty(NAME,IDX)                                                                \
  void Set##NAME##Cost(double aCost) { mCost(IDX) = aCost; }                                   \
  double NAME##Cost() const { return mCost(IDX); }                                             \

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
    const double aDurationS,
    const double aDurationD);

  TTrajectory(double end_s_d, double end_d, double delta_s, double mDurationS, double mDurationD);

  static TTrajectoryPtr SVelocityKeepingTrajectory(
    const Eigen::VectorXd& aStartState,
    double aCurrentTime,
    double aTargetVelocity,
    double aTargetD,
    double aDurationS,
    double aDurationD);

  Eigen::VectorXd EvalAt(double t) const;

  double AccelerationCost(double aHorizonTime) const;
  double JerkCost(double aHorizonTime) const;
  double SafetyDistanceCost(const Eigen::MatrixXd& aTrajectory, double aHorizonTime) const;

  double MinVelocity() const;
  double MaxVelocity() const;

  double VelocityCost(double aTargetVeloctiy, double aHorizonTime) const;
  double LaneOffsetCost(double aDuration) const;
  double TargetLaneCost() const;


  double DurationS() const;
  double DurationD() const;
  double TargetD() const;

  MCostProperty(Velocity, 0)
  MCostProperty(Acceleration, 1)
  MCostProperty(Jerk, 2)
  MCostProperty(SafetyDistance, 3)
  MCostProperty(Time, 4)
  MCostProperty(LaneOffset, 5)
  MCostProperty(Lane, 6)


  double Cost() const;

public:
  static double SDerivCoeff(int idx, int deriv);
  static double SEvalAt(const Eigen::VectorXd& c, double t, int deriv);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, double t);
  static Eigen::VectorXd SEvalStateAt(const Eigen::VectorXd& s_coeffs, const Eigen::VectorXd& d_coeffs, double t);
  static double SLongitudinalSafetyDistanceCost(double aDistance, double aVelocity);
  static double SLateralSafetyDistanceCost(double aDistance);

  void PrintCost() const;

private:
  Eigen::VectorXd mStartState;
  Eigen::VectorXd mEndState;
  Eigen::VectorXd mSCoeffs;
  Eigen::VectorXd mDCoeffs;
  bool mIsFinalized{false};
  double mDurationS{0.0};
  double mDurationD{0.0};
  double mStartTime{0.0};
  double mTimeStep{0.02};
  double mCostDeltaT{0.1};

  Eigen::VectorXd mCost{Eigen::VectorXd::Zero(7)};
};

//==============================================================================================
#endif //TRAJECTORY_H
//==============================================================================================
