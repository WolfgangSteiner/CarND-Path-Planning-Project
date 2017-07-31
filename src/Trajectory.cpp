//==============================================================================================
#include "Trajectory.h"
//==============================================================================================
#include "Eigen-3.3/Eigen/Eigen"
#include <assert.h>
//==============================================================================================
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
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

  MatrixXd M = MatrixXd(3,3);
  M <<   T3,    T4,    T5,
       3*T2,  4*T3,  5*T4,
        6*T, 12*T2, 20*T3;

  VectorXd s = VectorXd(3);
  s << s_f - (s_i + s_i_d * T + 0.5 * s_i_dd * T2),
       s_f_d - (s_i_d + s_i_dd * T),
       s_f_dd - s_i_dd;

  VectorXd coeffs = VectorXd(6);
  coeffs << s_i, s_i_d, 0.5*s_i_dd, M.inverse() * s;
  return coeffs;
}

//----------------------------------------------------------------------------------------------

double Trajectory::SDerivCoeff(int idx, int deriv)
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

double Trajectory::SEvalAt(const VectorXd& c, double t, int deriv)
{
  double t_pow = 1.0;
  double s = 0.0;

  for (int i = deriv; i < 6; ++i)
  {
    s += c(i) * Trajectory::SDerivCoeff(i, deriv) * t_pow;
    t_pow *= t;
  }

  return s;
}


//----------------------------------------------------------------------------------------------

VectorXd Trajectory::SEvalStateAt(const VectorXd& c, double t)
{
  VectorXd state = VectorXd::Zero(3);
  for (int i = 0; i < 3; ++i)
  {
    state(i) = SEvalAt(c, t, i);
  }

  return state;
}


//----------------------------------------------------------------------------------------------

VectorXd Trajectory::SEvalStateAt(const Eigen::VectorXd& s_coeffs, const Eigen::VectorXd& d_coeffs, double t)
{
  VectorXd state = VectorXd::Zero(6);
  state << SEvalStateAt(s_coeffs, t), SEvalStateAt(d_coeffs, t);
  return state;
}


//==============================================================================================

Trajectory::Trajectory(
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

Trajectory::Trajectory(double end_s_d, double end_d, double delta_s, double delta_t)
: mStartTime(0.0)
, mDuration(delta_t)
{
  mStartState = VectorXd::Zero(6);
  mEndState = VectorXd(6);
  mEndState << delta_s, end_s_d, 0.0, end_d, 0.0, 0.0;
}


//----------------------------------------------------------------------------------------------

void Trajectory::Finalize(const Eigen::VectorXd& aStartState, double aStartTime)
{
  mStartState = aStartState;
  mEndState(0) += mStartState(0);
  mStartTime = aStartTime;
  mSCoeffs = SCalcTrajectoryCoefficients(mStartState.head(3), mEndState.head(3), mDuration);
  mDCoeffs = SCalcTrajectoryCoefficients(mStartState.segment(3,3), mEndState.segment(3,3), mDuration);
  mIsFinalized = true;
}


//----------------------------------------------------------------------------------------------

Eigen::VectorXd Trajectory::EvalAt(double t) const
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

std::vector<Eigen::VectorXd> Trajectory::GetTrajectory() const
{
  vector<Eigen::VectorXd> trajectory;

  for (double t = 0; t < mDuration; t += dt_)
  {
    trajectory.push_back(SEvalStateAt(mSCoeffs, mDCoeffs, t));
  }

  return trajectory;
}


//----------------------------------------------------------------------------------------------

bool Trajectory::IsFinished(double t) const
{
  return t >= mStartTime + mDuration;
}


//==============================================================================================
