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

static double SSafetyDistanceCost(double aDistance, double aVelocity)
{
  const double kSafetyDistance = aVelocity * 3.6 / 2.0;
  if (aDistance > 1.25 * kSafetyDistance)
  {
    return 0.0;
  }
  else
  {
    return std::exp(-aDistance / (-kSafetyDistance/std::log(0.1)));
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
  A << 2 * T2, 4 * T3, 6 * T, 12 * T3;

  VectorXd b(2);
  b << aTargetVelocity - aStartState(0) - aStartState(2) * T,
       0.0 - aStartState(2);

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  std::cout << "x: " << x << std::endl;

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
, mDuration{0.0}
, mStartTime{0.0}
{
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectory(
  const Eigen::VectorXd& start_state,
  const Eigen::VectorXd& end_state,
  const double start_t,
  const double duration)
: mStartTime(start_t)
, mDuration(duration)
, mIsFinalized(true)
{
  mSCoeffs = SCalcTrajectoryCoefficients(start_state.head(3), end_state.head(3), duration);
  mDCoeffs = SCalcTrajectoryCoefficients(start_state.segment(3,3), end_state.segment(3,3), duration);
}

//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectory(double end_s_d, double end_d, double delta_s, double delta_t)
: mStartTime(0.0)
, mDuration(delta_t)
{
  mStartState = VectorXd::Zero(6);
  mEndState = VectorXd(6);
  mEndState << delta_s, end_s_d, 0.0, end_d, .00, 0.0;
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectory::SConstantVelocityTrajectory(
  double aCurrentS, double aCurrentD, double aVelocity, double aCurrentTime, double aDuration)
{
  TTrajectoryPtr pTrajectory(new TTrajectory());
  pTrajectory->mStartState << aCurrentS, aVelocity, 0.0, aCurrentD, 0.0, 0.0;
  pTrajectory->mEndState << aCurrentS + aVelocity * aDuration, aVelocity, 0.0, aCurrentD, 0.0, 0.0;
  pTrajectory->mSCoeffs << aCurrentS, aVelocity, 0.0, 0.0, 0.0, 0.0;
  pTrajectory->mDCoeffs << aCurrentD, 0.0, 0.0, 0.0, 0.0, 0.0;
  pTrajectory->mIsFinalized = true;
  pTrajectory->mStartTime = aCurrentTime;
  pTrajectory->mDuration = aDuration;

  return pTrajectory;
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectory::SVelocityKeepingTrajectory(
  const Eigen::VectorXd& aStartState, double aTargetVelocity, double aCurrentTime, double aDuration)
{
  TTrajectoryPtr pTrajectory(new TTrajectory());
  pTrajectory->mStartState = aStartState;
  pTrajectory->mStartTime = aCurrentTime;
  pTrajectory->mDuration = aDuration;

  pTrajectory->mSCoeffs = SCalcVelocityKeepingSCoefficients(aStartState,aTargetVelocity,aDuration);
  pTrajectory->mDCoeffs(0) = aStartState(3);
  pTrajectory->mIsFinalized = true;

  return pTrajectory;
}


//----------------------------------------------------------------------------------------------

void TTrajectory::Finalize(const Eigen::VectorXd& aStartState, double aStartTime)
{
  mStartState = aStartState;
  mEndState(0) += mStartState(0);
  mStartTime = aStartTime;
  mSCoeffs = SCalcTrajectoryCoefficients(mStartState.head(3), mEndState.head(3), mDuration);
  mDCoeffs = SCalcTrajectoryCoefficients(mStartState.segment(3,3), mEndState.segment(3,3), mDuration);

  std::cout << "mStartState: " << mStartState << std::endl;
  std::cout << "mEndState: " << mEndState << std::endl;
  std::cout << "mSCoeffs: " << mSCoeffs << std::endl;

  mIsFinalized = true;
}


//----------------------------------------------------------------------------------------------

Eigen::VectorXd TTrajectory::EvalAt(double t) const
{
  assert(mIsFinalized);

  if (t > mStartTime + mDuration)
  {
    VectorXd state = SEvalStateAt(mSCoeffs, mDCoeffs, mDuration);
    VectorXd speed_vector = VectorXd::Zero(6);
    speed_vector(0) = state(1);
    speed_vector(3) = state(4);
    state += (t - mStartTime - mDuration) * speed_vector;
    return state;
  }
  else
  {
    return SEvalStateAt(mSCoeffs, mDCoeffs, t - mStartTime);
  }
}


//----------------------------------------------------------------------------------------------

std::vector<Eigen::VectorXd> TTrajectory::GetTrajectory() const
{
  vector<Eigen::VectorXd> trajectory;

  for (double t = 0; t < mDuration; t += mTimeStep)
  {
    trajectory.push_back(SEvalStateAt(mSCoeffs, mDCoeffs, t));
  }

  return trajectory;
}


//----------------------------------------------------------------------------------------------

bool TTrajectory::IsFinished(double t) const
{
  return t > mStartTime + mDuration;
}


//----------------------------------------------------------------------------------------------

std::tuple<double,double> TTrajectory::MinDistanceToTrajectory(const TTrajectoryPtr apOtherTrajectory) const
{
  double t = mStartTime;
  double MinDist = 1.0e9;
  double MinDist_t = 0.0;

  while (t < mStartTime + mDuration)
  {
    Eigen::VectorXd s1 = EvalAt(t);
    Eigen::VectorXd s2 = apOtherTrajectory->EvalAt(t);
    const double Dist = NUtils::distance(s1(0), s1(3), s2(0), s2(3));
    if (Dist < MinDist)
    {
      MinDist = Dist;
      MinDist_t = t;
    }

    t += mTimeStep;
  }

  return std::make_tuple(MinDist, MinDist_t);
}

//----------------------------------------------------------------------------------------------

void TTrajectory::AddCost(double c)
{
  mCost += c;
  assert(mCost == mCost);
}


//----------------------------------------------------------------------------------------------

double TTrajectory::Cost() const
{
  return mCost;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::JerkCost() const
{
  double Cs = 0.0;
  double Cd = 0.0;

  for (double t = 0; t < mDuration; t+=mTimeStep)
  {
    const double Js = SEvalAt(mSCoeffs, t, 3);
    const double Jd = SEvalAt(mDCoeffs, t, 3);
    assert(Js == Js);
    assert(Jd == Jd);
    Cs += Js*Js;
    Cd += Jd*Jd;
  }

  return Cs + 2.0 * Cd;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::SafetyDistanceCost(const TTrajectoryPtr apOtherTrajectory) const
{
  double kMinDist, kMinDist_t;
  std::tie(kMinDist, kMinDist_t) = MinDistanceToTrajectory(apOtherTrajectory);
  const double kMinDistSpeed = SEvalAt(mSCoeffs, kMinDist_t, 1);

  return SSafetyDistanceCost(kMinDist, kMinDistSpeed);
}


//----------------------------------------------------------------------------------------------

double TTrajectory::MinVelocity() const
{
  double MinVelocity = 1e9;

  for (double t = 0; t < mDuration; t+=mTimeStep)
  {
    MinVelocity = std::min(MinVelocity, SEvalAt(mSCoeffs, t, 1));
  }

  return MinVelocity;
}


//----------------------------------------------------------------------------------------------

double TTrajectory::MaxVelocity() const
{
  double MaxVelocity = -1e9;

  for (double t = 0; t < mDuration; t+=mTimeStep)
  {
    MaxVelocity = std::max(MaxVelocity, SEvalAt(mSCoeffs, t, 1));
  }

  return MaxVelocity;
}


//==============================================================================================
