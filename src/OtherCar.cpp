//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "OtherCar.h"
#include "Trajectory.h"
#include "Utils.h"
//==============================================================================================

TOtherCar::TOtherCar()
{}


//----------------------------------------------------------------------------------------------

TOtherCar::TOtherCar(double aS, double aD, double aVs)
: mKalmanFilter(aS, aD, aVs)
{}


//----------------------------------------------------------------------------------------------

bool TOtherCar::IsInLane(int aLaneNumber) const
{
  return NUtils::SLaneNumberForD(D()) == aLaneNumber;
}


//----------------------------------------------------------------------------------------------

void TOtherCar::Update(double aS, double aD, double aVs)
{
  mKalmanFilter.Update(aS, aD, aVs);
}


//----------------------------------------------------------------------------------------------

void TOtherCar::Predict(double aDeltaT)
{
  mKalmanFilter.Predict(aDeltaT);
}


//----------------------------------------------------------------------------------------------

double TOtherCar::S() const
{
  return mKalmanFilter.S();
}


//----------------------------------------------------------------------------------------------

double TOtherCar::D() const
{
  return mKalmanFilter.D();
}


//----------------------------------------------------------------------------------------------

double TOtherCar::Velocity() const
{
  return mKalmanFilter.Vs();
}


//----------------------------------------------------------------------------------------------

Eigen::MatrixXd TOtherCar::CurrentTrajectory(double aDeltaT, double aDuration)
{
  mKalmanFilter.PushState();
  const int n = int(aDuration / aDeltaT);
  Eigen::MatrixXd Result(n, 4);

  for (int i = 0; i < n; ++i)
  {
    Result.row(i) << mKalmanFilter.X().transpose();
    mKalmanFilter.Predict(aDeltaT);
  }

  mKalmanFilter.PopState();

  return Result;
}


//==============================================================================================
