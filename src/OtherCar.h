//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef OTHERCAR_H
#define OTHERCAR_H
//==============================================================================================
#include "KalmanFilter.h"
#include "Trajectory.h"
//==============================================================================================
#include <vector>
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

class TOtherCar
{
public:
  TOtherCar();
  TOtherCar(double aS, double aD, double aVs);

public:
  bool IsInLane(int aLaneNumber) const;
  double S() const;
  double D() const;

  double Velocity() const;
  Eigen::MatrixXd CurrentTrajectory(double aDeltaT, double aDuration);

  void Update(double aS, double aD, double aVs);
  void Predict(double aDeltaT);

private:
  TKalmanFilter mKalmanFilter;
  Eigen::VectorXd mX;
};


//==============================================================================================
#endif // OTHERCAR_H
//==============================================================================================
