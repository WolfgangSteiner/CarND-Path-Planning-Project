//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "SensorFusion.h"
#include "OtherCar.h"
#include "Utils.h"
//==============================================================================================

TSensorFusion::TSensorFusion(const std::vector<std::vector<double>>& aSensorFusionData)
{
  for (const auto& v : aSensorFusionData)
  {
    mOtherCars.push_back(TOtherCar(v));
  }
}


//==============================================================================================

std::list<TOtherCar> TSensorFusion::OtherLeadingCarsInLane(const Eigen::VectorXd& aState) const
{
  const double kCurrentS = aState[0];
  const double kCurrentD = aState[3];
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);

  std::list<TOtherCar> Result;

  for (const TOtherCar& iOtherCar : mOtherCars)
  {
    if (iOtherCar.IsInLane(kCurrentLane) && iOtherCar.S() > kCurrentS)
    {
      Result.push_back(iOtherCar);
    }
  }

  Result.sort([](const TOtherCar& CarA, const TOtherCar& CarB) { return CarA.S() > CarB.S(); });

  return Result;
}


//==============================================================================================
