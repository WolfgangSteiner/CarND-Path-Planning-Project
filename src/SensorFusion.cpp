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

TSensorFusion::TSensorFusion(
  const std::vector<std::vector<double>>& aSensorFusionData,
  const TWaypoints& aWaypoints,
  double aDelayT)
{
  for (const auto& iData : aSensorFusionData)
  {
    const int id = iData[0];
    const double vx = iData[3];
    const double vy = iData[4];
    const double v = sqrt(vx*vx + vy*vy);
    const double x = iData[1] + vx * aDelayT;
    const double y = iData[2] + vy * aDelayT;
    const double s = iData[5] + v * aDelayT;

    const Eigen::Vector2d p(x, y);
    const Eigen::Vector2d f = aWaypoints.CalcFrenet(p, s);
    mOtherCars.push_back(TOtherCar(id, x, y, vx, vy, f(0), f(1), v));
  }

//  std::cout << "Other cars at: ";
//  for (const auto& iCar : mOtherCars)
//    {
//    std::cout << "CHECK(check_transformation(" << iCar.X() << ", " << iCar.Y() <<  ", " << iCar.S() << "));" << std::endl;
//
//    //std::cout << "(" << iCar.X() << ", " << iCar.Y() << ", " << iCar.S() << "," << iCar.D() << "), ";
//  }
//  std::cout << std::endl;
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
    else
    {
      std::cout << "Ignoring other car at: (" << iOtherCar.S() << ", " << iOtherCar.D() << ")\n";
    }
  }

  Result.sort([](const TOtherCar& CarA, const TOtherCar& CarB) { return CarA.S() < CarB.S(); });

  for (const auto& iCar : Result)
  {
    std::cout << "Other car in front at: " << iCar.S() << std::endl;
  }

  return Result;
}


//==============================================================================================
