//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "OtherCar.h"
#include "Trajectory.h"
#include "Utils.h"
//==============================================================================================

TOtherCar::TOtherCar(const int aId, double aX, double aY, double aVx, double aVy, double aS, double aD, double aVelocity)
: mId{aId}
, mX{aX}
, mY{aY}
, mVx{aVx}
, mVy{aVy}
, mS{aS}
, mD{aD}
, mVelocity{aVelocity}
{}

//----------------------------------------------------------------------------------------------

bool TOtherCar::IsInLane(int aLaneNumber) const
{
  return NUtils::SLaneNumberForD(D()) == aLaneNumber;
}

//----------------------------------------------------------------------------------------------

double TOtherCar::S() const
{
  return mS;
}


//----------------------------------------------------------------------------------------------

double TOtherCar::D() const
{
  return mD;
}


//----------------------------------------------------------------------------------------------

double TOtherCar::X() const
{
  return mX;
}

//----------------------------------------------------------------------------------------------

double TOtherCar::Y() const
{
  return mY;
}


//----------------------------------------------------------------------------------------------

double TOtherCar::Velocity() const
{
  return mVelocity;
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TOtherCar::CurrentTrajectory(double aCurrentTime, double aDuration) const
{
  return TTrajectory::SConstantVelocityTrajectory(S(), D(), Velocity(), aCurrentTime, aDuration);
}


//==============================================================================================
