//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "OtherCar.h"
#include "Trajectory.h"
#include "Utils.h"
//==============================================================================================

TOtherCar::TOtherCar()
: mX{Eigen::VectorXd::Zero(4)}
{}


//----------------------------------------------------------------------------------------------

TOtherCar::TOtherCar(double aS, double aD, double aVs)
: mX{Eigen::VectorXd::Zero(4)}
{
  mX << aS, aD, aVs, 0.0;
}


//----------------------------------------------------------------------------------------------

bool TOtherCar::IsInLane(int aLaneNumber) const
{
  return NUtils::SLaneNumberForD(D()) == aLaneNumber;
}


//----------------------------------------------------------------------------------------------

void TOtherCar::Update(double aS, double aD, double aVs)
{
  mX << aS, aD, aVs, 0.0;
}


//----------------------------------------------------------------------------------------------

void TOtherCar::Predict(double aDeltaT)
{
  mX(0) += mX(2) * aDeltaT;
}


//----------------------------------------------------------------------------------------------

double TOtherCar::S() const
{
  return mX(0);
}


//----------------------------------------------------------------------------------------------

double TOtherCar::D() const
{
  return mX(1);
}


//----------------------------------------------------------------------------------------------

double TOtherCar::Velocity() const
{
  return mX(2);
}


//----------------------------------------------------------------------------------------------

Eigen::MatrixXd TOtherCar::CurrentTrajectory(double aDeltaT, double aDuration)
{
  const int n = int(aDuration / aDeltaT);
  Eigen::MatrixXd Result(n, 4);
  double s = mX(0);

  for (int i = 0; i < n; ++i)
  {
    s += mX(2) * aDeltaT;
    Result.row(i) << s, mX(1), mX(2), 0.0;
  }

  return Result;
}


//==============================================================================================
