//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef CHANGELANESTATE_H
#define CHANGELANESTATE_H
//==============================================================================================
#include "VehicleState.h"
//==============================================================================================

class TChangeLaneState : public TVehicleState
{
public:
  TChangeLaneState(double aTargetLane)
  : mTargetLane{aTargetLane}
  {}

  virtual ~TChangeLaneState() override {};

public:
  virtual std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) override;

private:
  double mTargetLane;
};


//==============================================================================================
#endif // CHANGELANESTATE_H
//==============================================================================================
