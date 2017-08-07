//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef SENSORFUSION_H
#define SENSORFUSION_H
//==============================================================================================
#include "OtherCar.h"
#include "Waypoints.h"
//==============================================================================================
#include <unordered_map>
#include <list>
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

class TSensorFusion
{
public:
  TSensorFusion();

  void Update(const std::vector<std::vector<double>>& aSensorFusionData, double aCurrentS);
  void Predict(double aDeltaT);

  std::list<TOtherCar> OtherLeadingCarsInLane(const Eigen::VectorXd& aState) const;
  std::list<TOtherCar> OtherNearbyCars(const Eigen::VectorXd& aState, double aDeltaS=200.0) const;

private:
  std::unordered_map<int,TOtherCar> mOtherCars;
  std::unordered_map<int,bool> mUpdateMap;
  TWaypoints mWaypoints;
};


//==============================================================================================
#endif // SENSORFUSION_H
//==============================================================================================
