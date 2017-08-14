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
  TChangeLaneState(
    TTrajectory::TTrajectoryPtr apLaneChangeTrajectory,
    int aStartLane,
    int aTargetLane)
  : mpLaneChangeTrajectory{apLaneChangeTrajectory}
  , mStartLane(aStartLane)
  , mTargetLane{aTargetLane}
  {}

  virtual ~TChangeLaneState() override {};

public:
  virtual std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) override;

private:
  int mStartLane;
  int mTargetLane;
  TTrajectory::TTrajectoryPtr mpLaneChangeTrajectory;
};


//==============================================================================================
#endif // CHANGELANESTATE_H
//==============================================================================================
