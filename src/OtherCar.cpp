//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "OtherCar.h"
#include "Trajectory.h"
#include "Utils.h"
//==============================================================================================

TOtherCar::TOtherCar(const std::vector<double>& aOtherCarState)
{
  mId = aOtherCarState[0];
  mState = Eigen::VectorXd(6);
  mState << aOtherCarState[1], aOtherCarState[2], aOtherCarState[3],
            aOtherCarState[4], aOtherCarState[5], aOtherCarState[6];
}

//----------------------------------------------------------------------------------------------

bool TOtherCar::IsInLane(int aLaneNumber) const
{
  return NUtils::SLaneNumberForD(D()) == aLaneNumber;
}

//----------------------------------------------------------------------------------------------

double TOtherCar::S() const
{
  return mState(4);
}


//----------------------------------------------------------------------------------------------

double TOtherCar::D() const
{
  return mState(5);
}


//----------------------------------------------------------------------------------------------

double TOtherCar::Velocity() const
{
  const double vx = mState(2);
  const double vy = mState(3);
  return sqrt(vx*vx + vy*vy);
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TOtherCar::CurrentTrajectory(double aCurrentTime, double aDuration) const
{
  return TTrajectory::SConstantVelocityTrajectory(S(), D(), Velocity(), aCurrentTime, aDuration);
}


//==============================================================================================
