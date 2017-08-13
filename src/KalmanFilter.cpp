//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "KalmanFilter.h"
//==============================================================================================
#include "Eigen-3.3/Eigen/Eigen"
//==============================================================================================
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
//==============================================================================================

TKalmanFilter::TKalmanFilter()
{
  mX = VectorXd::Zero(4);

  mP = MatrixXd::Identity(4,4);
  mP.diagonal() << 0.1, 0.1, 0.1, 10.0;

  mF = MatrixXd::Identity(4,4);
  mQ = MatrixXd::Zero(4,4);
  mR = MatrixXd::Identity(3,3) * 0.01;

  mH = MatrixXd(3,4);
  mH << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0;
}


//----------------------------------------------------------------------------------------------

TKalmanFilter::TKalmanFilter(double s, double d, double vs)
: TKalmanFilter()
{
  mX << s, d, vs, 0.0;
}


//----------------------------------------------------------------------------------------------

VectorXd TKalmanFilter::Predict(double dt)
{
  UpdateQ(dt);
  UpdateF(dt);

  mX = mF * mX;
  mP = mF * mP * mF.transpose() + mQ;

  return mX;
}


//----------------------------------------------------------------------------------------------

void TKalmanFilter::UpdateQ(double dt)
{
  const double dt_2 = dt * dt;
  const double dt_3 = dt_2 * dt;
  const double dt_4 = dt_3 * dt;
  const double var_a = 9; // as suggested by the project.

  mQ <<  dt_4/4*var_a, 0, dt_3/2*var_a, 0,
    0, dt_4/4*var_a, 0, dt_3/2*var_a,
    dt_3/2*var_a, 0, dt_2*var_a, 0,
    0, dt_3/2*var_a, 0, dt_2*var_a;
}


//----------------------------------------------------------------------------------------------

void TKalmanFilter::UpdateF(double dt)
{
  mF(0,2) = dt;
  mF(1,3) = dt;
}


//----------------------------------------------------------------------------------------------

void TKalmanFilter::Update(double s, double d, double vs)
{
  const Vector3d z(s,d,vs);
  const VectorXd z_pred = mH * mX;
  const VectorXd y = z - z_pred;
  const MatrixXd Ht = mH.transpose();
  const MatrixXd S = mH * mP * Ht + mR;
  const MatrixXd K = mP * Ht * S.inverse();

  //new estimate
  mX += (K * y);
  mP -= K * mH * mP;
}


//----------------------------------------------------------------------------------------------

double TKalmanFilter::S() const
{
  return mX(0);
}


//----------------------------------------------------------------------------------------------

double TKalmanFilter::D() const
{
  return mX(1);
}


//----------------------------------------------------------------------------------------------

double TKalmanFilter::Vs() const
{
  return mX(2);
}


//----------------------------------------------------------------------------------------------

const Eigen::VectorXd& TKalmanFilter::X() const
{
  return mX;
}


//----------------------------------------------------------------------------------------------

void TKalmanFilter::PushState()
{
  mStackX.push_back(mX);
  mStackP.push_back(mP);
}


//----------------------------------------------------------------------------------------------

void TKalmanFilter::PopState()
{
  mX = mStackX.back();
  mP = mStackP.back();

  mStackX.pop_back();
  mStackP.pop_back();
}

//==============================================================================================
