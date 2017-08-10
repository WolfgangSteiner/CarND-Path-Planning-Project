//==============================================================================================
#include "Trajectory.h"
#include "Utils.h"
//==============================================================================================
#include "Eigen-3.3/Eigen/Eigen"
#include <assert.h>
#include <iostream>
#include <tuple>
//==============================================================================================
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
//==============================================================================================

double TTrajectory::SLongitudinalSafetyDistanceCost(double aDistance, double aVelocity)
{
  const double kSafetyDistance = aVelocity * 3.6 / 3.0;
  static const double alpha = -2.0;
  static const double c_max = 1.0;
  static const double c_min = 0.05;
  static const double d0 = kSafetyDistance;
  static const double d1 = 10.0;
  static const double B = (c_max - c_min) / (std::exp(alpha * d1) - std::exp(alpha * d0));
  static const double A = c_max - B * std::exp(alpha * d1);

  if (aDistance > kSafetyDistance)
  {
    return 0.0;
  }
  else
  {
    return A + B * std::exp(alpha * aDistance);
  }
}


//----------------------------------------------------------------------------------------------

double TTrajectory::SLateralSafetyDistanceCost(double aDistance)
{
  const double kSafetyDistance = 3.75;
  const double kMinDistance = 2.0;
  static const double alpha = -2.0;
  static const double c_max = 1.0;
  static const double c_min = 0.05;
  static const double d0 = kSafetyDistance;
  static const double d1 = kMinDistance;
  static const double B = (c_max - c_min) / (std::exp(alpha * d1) - std::exp(alpha * d0));
  static const double A = c_max - B * std::exp(alpha * d1);

  if (aDistance >= kSafetyDistance)
  {
    return 0.0;
  }
  else
  {
    return A + B * std::exp(alpha * aDistance);
  }
}


//----------------------------------------------------------------------------------------------

static double SSafetyDistanceCost(
  double aLongitudinalDistance,
  double aLateralDistance,
  double aVelocity)
{
  const double kLongitudinalDistanceCost = TTrajectory::SLongitudinalSafetyDistanceCost(aLongitudinalDistance, aVelocity);
  const double kLateralDistanceCost = TTrajectory::SLateralSafetyDistanceCost(aLateralDistance);

  if (aLongitudinalDistance < 4.5)
  {
    if (aLateralDistance > 2.0)
    {
      return kLateralDistanceCost;
    }
    else
    {
      // We have already crashed!
      return 1000;
    }
  }
  else if (aLateralDistance > 3.5)
  {
    // Other car is on adjacent lane.
    return 0.0;
  }
  else
  {
    return kLongitudinalDistanceCost;
  }
}


//----------------------------------------------------------------------------------------------

static double SLaneOffsetCost(double aD)
{
  const int kLaneNumber = NUtils::SLaneNumberForD(aD);
  const double kLaneCenter = NUtils::SDForLaneNumber(kLaneNumber);
  const double kDistanceToLaneCenter = NUtils::SDistance(kLaneCenter, aD);
  static const double alpha = 4.0;
  static const double c_max = 1.0;
  static const double c_min = 0.05;
  static const double d0 = 0.25;
  static const double d1 = 0.5 * NUtils::SLaneWidth();
  static const double B = (c_max - c_min) / (std::exp(alpha * d1) - std::exp(alpha * d0));
  static const double A = c_max - B * std::exp(alpha * d1);

  if (kDistanceToLaneCenter <= 0.25)
  {
    return 0.0;
  }
  else
  {
    return A + B * std::exp(alpha * kDistanceToLaneCenter);
  }
}


//----------------------------------------------------------------------------------------------

static double SVelocityCost(double vs, double vd, double vmax)
{
  const double v = sqrt(vs*vs + vd*vd);

  if (v >= vmax)
  {
    return 1e6;
  }
  else
  {
    return pow(vmax - v, 2);
  }
}


//==============================================================================================

static VectorXd SCalcTrajectoryCoefficients(
  const VectorXd& start_state,
  const VectorXd& end_state,
  double T)
{
  const double s_i = start_state[0];
  const double s_i_d = start_state[1];
  const double s_i_dd = start_state[2];

  const double s_f = end_state[0];
  const double s_f_d = end_state[1];
  const double s_f_dd = end_state[2];

  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  MatrixXd A = MatrixXd(3,3);
  A <<   T3,    T4,    T5,
       3*T2,  4*T3,  5*T4,
        6*T, 12*T2, 20*T3;

  VectorXd b = VectorXd(3);
  b << s_f - (s_i + s_i_d * T + 0.5 * s_i_dd * T2),
       s_f_d - (s_i_d + s_i_dd * T),
       s_f_dd - s_i_dd;

  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

  VectorXd coeffs = VectorXd(6);
  coeffs << s_i, s_i_d, 0.5*s_i_dd, x;
  return coeffs;
}


//----------------------------------------------------------------------------------------------

static VectorXd SCalcVelocityKeepingSCoefficients(
  const VectorXd& aStartState,
  double aTargetVelocity,
  double aDuration)
{
  const double T = aDuration;
  const double T2 = T * T;
  const double T3 = T2 * T;

  MatrixXd A(2,2);
  A << 3 * T2, 4 * T3, 6 * T, 12 * T2;

  VectorXd b(2);
  b << aTargetVelocity - aStartState(1) - aStartState(2) * T,
       0.0 - aStartState(2);

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  //std::cout << "x: " << x << std::endl;

  assert(x(0) == x(0));
  assert(x(1) == x(1));

  VectorXd SParams(6);
  SParams << aStartState(0), aStartState(1), 0.5 * aStartState(2), x, 0.0;
  return SParams;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::SDerivCoeff(int idx, int deriv)
{
  if (deriv > idx)
  {
    return 0.0;
  }
  else
  {
    double result = 1.0;
    for (int i = idx; i > idx - deriv; i--)
    {
      result *= i;
    }
    return result;
  }
}


//----------------------------------------------------------------------------------------------

double TTrajectory::SEvalAt(const VectorXd& c, double t, int deriv)
{
  double t_pow = 1.0;
  double s = 0.0;

  for (int i = deriv; i < 6; ++i)
  {
    s += c(i) * TTrajectory::SDerivCoeff(i, deriv) * t_pow;
    t_pow *= t;
  }

  return s;
}


//----------------------------------------------------------------------------------------------

VectorXd TTrajectory::SEvalStateAt(const VectorXd& c, double t)
{
  VectorXd state = VectorXd::Zero(3);
  for (int i = 0; i < 3; ++i)
  {
    state(i) = SEvalAt(c, t, i);
  }

  return state;
}


//----------------------------------------------------------------------------------------------

VectorXd TTrajectory::SEvalStateAt(const Eigen::VectorXd& s_coeffs, const Eigen::VectorXd& d_coeffs, double t)
{
  VectorXd state = VectorXd::Zero(6);
  state << SEvalStateAt(s_coeffs, t), SEvalStateAt(d_coeffs, t);
  return state;
}



//==============================================================================================

TTrajectory::TTrajectory()
: mStartState{VectorXd::Zero(6)}
, mEndState{VectorXd::Zero(6)}
, mSCoeffs{VectorXd::Zero(6)}
, mDCoeffs{VectorXd::Zero(6)}
, mIsFinalized{false}
, mDurationS{0.0}
, mDurationD{0.0}
, mStartTime{0.0}
{
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectory(
  const Eigen::VectorXd& start_state,
  const Eigen::VectorXd& end_state,
  const double start_t,
  const double aDurationS,
  const double aDurationD)
: mStartTime(start_t)
, mDurationD(aDurationS)
, mDurationS(aDurationD)
, mIsFinalized(true)
{
  mSCoeffs = SCalcTrajectoryCoefficients(start_state.head(3), end_state.head(3), mDurationS);
  mDCoeffs = SCalcTrajectoryCoefficients(start_state.segment(3,3), end_state.segment(3,3), mDurationD);
}

//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectory(double end_s_d, double end_d, double delta_s, double aDurationS, double aDurationD)
: mStartTime(0.0)
, mDurationS(aDurationS)
, mDurationD(aDurationD)
{
  mStartState = VectorXd::Zero(6);
  mEndState = VectorXd(6);
  mEndState << delta_s, end_s_d, 0.0, end_d, .00, 0.0;
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectory::SVelocityKeepingTrajectory(
  const Eigen::VectorXd& aStartState,
  double aCurrentTime,
  double aTargetVelocity,
  double aTargetD,
  double aDurationS,
  double aDurationD)
{
  TTrajectoryPtr pTrajectory(new TTrajectory());
  pTrajectory->mStartState = aStartState;
  pTrajectory->mStartTime = aCurrentTime;
  pTrajectory->mEndState(3) = aTargetD;
  pTrajectory->mDurationS = aDurationS;
  pTrajectory->mDurationD = aDurationD;

  pTrajectory->mSCoeffs = SCalcVelocityKeepingSCoefficients(aStartState, aTargetVelocity, aDurationS);

  const double kCurrentD = aStartState(3);
  if (kCurrentD == aTargetD)
  {
    pTrajectory->mDCoeffs(0) = aStartState(3);
  }
  else
  {
    pTrajectory->mDCoeffs = SCalcTrajectoryCoefficients(
      pTrajectory->mStartState.segment(3,3),
      pTrajectory->mEndState.segment(3,3),
      aDurationD);
  }

  pTrajectory->mIsFinalized = true;

  return pTrajectory;
}


//----------------------------------------------------------------------------------------------

Eigen::VectorXd TTrajectory::EvalAt(double t) const
{
  assert(mIsFinalized);
  t -= mStartTime;

  VectorXd state_s = SEvalStateAt(mSCoeffs, std::min(t, mDurationS));
  VectorXd state_d = SEvalStateAt(mDCoeffs, std::min(t, mDurationD));

  if (t > mDurationS)
  {
    state_s(0) += state_s(1) * (t - mDurationS);
  }

  if (t > mDurationD)
  {
    state_d(0) += state_d(1) * (t - mDurationD);
  }

  VectorXd state(6);
  state << state_s, state_d;

  return state;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::DurationS() const
{
  return mDurationS;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::DurationD() const
{
  return mDurationS;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::TargetD() const
{
  return mEndState(3);
}


//----------------------------------------------------------------------------------------------

double TTrajectory::JerkCost(double aHorizonTime) const
{
  double Cost = 0.0;
  const double n = aHorizonTime / mCostDeltaT;

  for (double t = 0; t < aHorizonTime; t+=mCostDeltaT)
  {
    const double Js = SEvalAt(mSCoeffs, std::min(t, mDurationS), 3);
    const double Jd = SEvalAt(mDCoeffs, std::min(t, mDurationD), 3);
    assert(Js == Js);
    assert(Jd == Jd);
    Cost += (Jd*Jd + Js*Js);
  }

  return Cost / n;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::AccelerationCost(double aHorizonTime) const
{
  double Cost = 0.0;
  const double n = aHorizonTime / mCostDeltaT;

  for (double t = 0; t < aHorizonTime; t+=mCostDeltaT)
  {
    const double As = SEvalAt(mSCoeffs, std::min(t, mDurationS), 2);
    const double Ad = SEvalAt(mDCoeffs, std::min(t, mDurationD), 2);
    Cost += (Ad*Ad + As*As);
  }

  return Cost / n;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::VelocityCost(double aTargetVelocity, double aHorizonTime) const
{
  double Cost = 0.0;

  double t = 0.0;
  const int n = int(aHorizonTime / mCostDeltaT);

  for (int i = 0; i < n; ++i)
  {
    const double vs = SEvalAt(mSCoeffs, std::min(t, mDurationS), 1);
    const double vd = SEvalAt(mDCoeffs, std::min(t, mDurationD), 1);

    Cost += SVelocityCost(vs, vd, aTargetVelocity);
    t += mCostDeltaT;
  }

  return Cost / n;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::SafetyDistanceCost(
  const MatrixXd& s2,
  double aDuration) const
{
  double Cost = 0.0;

  double t = mStartTime;
  const int n = int(aDuration / mCostDeltaT);

  for (int i = 0; i < n; ++i)
  {
    const Eigen::VectorXd s1 = EvalAt(t);
    const double kLongitudinalDist = NUtils::SDistance(s1(0), s2(i, 0));
    const double kLateralDist = NUtils::SDistance(s1(3), s2(i, 1));
    const double kSpeed = s1(1);

    Cost += SSafetyDistanceCost(kLongitudinalDist, kLateralDist, kSpeed);

    t += mCostDeltaT;
  }

  return Cost / n;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::LaneOffsetCost(double aDuration) const
{
  double Cost = 0.0;

  double t = 0.0;
  const int n = int(aDuration / mCostDeltaT);

  for (int i = 0; i < n; ++i)
  {
    const double iD = SEvalAt(mDCoeffs, std::min(t, mDurationD), 0);
    Cost += SLaneOffsetCost(iD);

    t += mCostDeltaT;
  }

  return Cost / n;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::MinVelocity() const
{
  double MinVelocity = 1e9;

  for (double t = 0; t < mDurationS; t+=mTimeStep)
  {
    MinVelocity = std::min(MinVelocity, SEvalAt(mSCoeffs, t, 1));
  }

  return MinVelocity;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::MaxVelocity() const
{
  double MaxVelocity = -1e9;

  for (double t = 0; t < mDurationS; t+=mTimeStep)
  {
    MaxVelocity = std::max(MaxVelocity, SEvalAt(mSCoeffs, t, 1));
  }

  return MaxVelocity;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::Cost() const
{
  for (int i = 0; i < mCost.rows(); ++i)
  {
    assert(mCost(i) == mCost(i));
  }

  return mCost.sum();
}



//----------------------------------------------------------------------------------------------

void TTrajectory::PrintCost() const
{
  std::cout
    << " v_min: "  << MinVelocity()
    << " v_max "   << MaxVelocity()
    << " DC: "     << SafetyDistanceCost()
    << " VC: "     << VelocityCost()
    << " AC: "     << AccelerationCost()
    << " JC: "     << JerkCost()
    << " LC: "     << LaneOffsetCost()
    << " TC: "     << TimeCost()
    << " C: "      << Cost()
    << std::endl;
}


//==============================================================================================
