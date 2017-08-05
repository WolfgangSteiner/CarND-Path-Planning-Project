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

private:
  double mMaxVelocity{22};
  double mVelocityCostFactor{1.0};
  double mJerkCostFactor{0.01};
  double mTimeCostFactor{100};
  double mSafetyDistanceFactor{1000};
};


//==============================================================================================
#endif // KEEPLANESTATE_H
//==============================================================================================