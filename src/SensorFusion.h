//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef SENSORFUSION_H
#define SENSORFUSION_H
//==============================================================================================
#include "OtherCar.h"
//==============================================================================================
#include <vector>
#include <list>
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

class TSensorFusion
{
public:
  TSensorFusion(const std::vector<std::vector<double>>& aSensorFusionData);
  std::list<TOtherCar> OtherLeadingCarsInLane(const Eigen::VectorXd& aState) const;

private:
  std::vector<TOtherCar> mOtherCars;
};


//==============================================================================================
#endif // SENSORFUSION_H
//==============================================================================================
