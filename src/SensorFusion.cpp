//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "SensorFusion.h"
#include "OtherCar.h"
#include "Waypoints.h"
#include "Utils.h"
//==============================================================================================
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================
using Eigen::Vector2d;
//==============================================================================================

TSensorFusion::TSensorFusion()
{}


//==============================================================================================

void TSensorFusion::Update(
  const std::vector<std::vector<double>>& aSensorFusionData,
  double aCurrentS)
{
  mUpdateMap.clear();

  for (const auto& iElement : mOtherCars)
  {
    mUpdateMap[iElement.first] = false;
  }

  for (const auto& iData : aSensorFusionData)
  {
    const int id = iData[0];
    const double vx = iData[3];
    const double vy = iData[4];
    const double v = sqrt(vx*vx + vy*vy);
    const double x = iData[1];
    const double y = iData[2];
    const double s0 = iData[5];
    const Eigen::Vector2d f = mWaypoints.CalcFrenet(Vector2d(x,y), s0);

    double s = f(0);
    const double kMaxS = mWaypoints.MaxS();

    while (aCurrentS - s > kMaxS / 2.0)
    {
      s += kMaxS;
    }

    while (aCurrentS - s < -kMaxS / 2.0)
    {
      s -= kMaxS;
    }

    auto FindIter = mOtherCars.find(id);

    if (FindIter == mOtherCars.end())
    {
      mOtherCars[id] = TOtherCar(s, f(1), v);
      mUpdateMap[id] = true;
    }
    else
    {
      mOtherCars[id].Update(s, f(1), v);
      mUpdateMap[id] = true;
    }
  }

  for (const auto& iElement : mUpdateMap)
  {
    if (iElement.second == false)
    {
      mOtherCars.erase(iElement.first);
    }
  }
}


//----------------------------------------------------------------------------------------------

void TSensorFusion::Predict(double aDeltaT)
{
  for (auto& iElement : mOtherCars)
  {
    iElement.second.Predict(aDeltaT);
  }
}


//==============================================================================================

std::list<TOtherCar> TSensorFusion::OtherLeadingCarsInLane(const Eigen::VectorXd& aState) const
{
  const double kCurrentS = aState[0];
  const double kCurrentD = aState[3];
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);

  std::list<TOtherCar> Result;

  for (const auto& iPair  : mOtherCars)
  {
    const auto& iOtherCar = iPair.second;
    if (iOtherCar.IsInLane(kCurrentLane) && iOtherCar.S() > kCurrentS)
    {
      Result.push_back(iOtherCar);
    }
  }

  Result.sort([](const TOtherCar& CarA, const TOtherCar& CarB) { return CarA.S() < CarB.S(); });

  return Result;
}

//----------------------------------------------------------------------------------------------

std::list<TOtherCar> TSensorFusion::OtherNearbyCarsInLane(
  const Eigen::VectorXd& aState,
  int aTargetLane,
  double aDeltaS) const
{
  const double kCurrentS = aState(0);

  std::list<TOtherCar> CarsInLane;

  for (const auto& iPair : mOtherCars)
  {
    const auto &iOtherCar = iPair.second;
    if (iOtherCar.IsInLane(aTargetLane) && NUtils::SDistance(iOtherCar.S(), aState(0)) < 200)
    {
      CarsInLane.push_back(iOtherCar);
    }
  }

  return CarsInLane;
}


//----------------------------------------------------------------------------------------------

std::vector<Eigen::MatrixXd> TSensorFusion::LeadingVehicleTrajectoriesInLane(
  const Eigen::VectorXd& aState,
  double aDeltaT,
  double aDuration) const
{
  auto LeadingCars = OtherLeadingCarsInLane(aState);
  return GetTrajectoriesForOtherCars(LeadingCars, aDeltaT, aDuration);
}


//----------------------------------------------------------------------------------------------

std::vector<Eigen::MatrixXd> TSensorFusion::OtherVehicleTrajectoriesInTargetLane(
  const Eigen::VectorXd& aState,
  int aTargetLane,
  double aDeltaT,
  double aDuration) const
{
  auto OtherCars = OtherNearbyCarsInLane(aState, aTargetLane);
  return GetTrajectoriesForOtherCars(OtherCars, aDeltaT, aDuration);
}


//----------------------------------------------------------------------------------------------

std::vector<Eigen::MatrixXd> TSensorFusion::GetTrajectoriesForOtherCars(
  std::list<TOtherCar>& aOtherCars,
  double aDeltaT,
  double aDuration) const
{
  std::vector<Eigen::MatrixXd> Trajectories;
  for (TOtherCar& iOtherCar : aOtherCars)
  {
    Trajectories.push_back(iOtherCar.CurrentTrajectory(aDeltaT, aDuration));
  }
  return Trajectories;
}


//==============================================================================================
