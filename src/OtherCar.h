//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef OTHERCAR_H
#define OTHERCAR_H
//==============================================================================================
#include "Trajectory.h"
//==============================================================================================
#include <vector>
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

class TOtherCar
{
public:
  TOtherCar(const std::vector<double>& aOtherCarState);

public:
  bool IsInLane(int aLaneNumber) const;
  double S() const;
  double D() const;
  double Velocity() const;
  TTrajectory::TTrajectoryPtr CurrentTrajectory(double aCurrentTime, double aDuration) const;

private:
  int mId;
  Eigen::VectorXd mState;
};


//==============================================================================================
#endif // OTHERCAR_H
//==============================================================================================
