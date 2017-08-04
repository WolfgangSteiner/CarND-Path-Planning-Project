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
  TOtherCar(const int aId, double aX, double aY, double aVx, double aVy, double aS, double aD, double aVelocity);

public:
  bool IsInLane(int aLaneNumber) const;
  double S() const;
  double D() const;
  double X() const;
  double Y() const;

  double Velocity() const;
  TTrajectory::TTrajectoryPtr CurrentTrajectory(double aCurrentTime, double aDuration) const;

private:
  int mId;
  double mS{0.0};
  double mD{0.0};
  double mX{0.0};
  double mY{0.0};
  double mVx{0.0};
  double mVy{0.0};
  double mVelocity{0.0};
};


//==============================================================================================
#endif // OTHERCAR_H
//==============================================================================================
