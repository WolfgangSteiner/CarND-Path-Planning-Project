//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef KEEPLANESTATE_H
#define KEEPLANESTATE_H
//==============================================================================================
#include "VehicleState.h"
//==============================================================================================

class TKeepLaneState : public TVehicleState
{
public:
  TKeepLaneState() {};
  virtual ~TKeepLaneState() override {};

public:
  virtual TTrajectory::TTrajectoryPtr Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) override;

  virtual TVehicleState* NextVehicleState(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) override;
};


//==============================================================================================
#endif // KEEPLANESTATE_H
//==============================================================================================
