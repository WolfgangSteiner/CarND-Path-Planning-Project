//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef KEEPLANESTATE_H
#define KEEPLANESTATE_H
//==============================================================================================
#include "VehicleState.h"
//==============================================================================================
class TTrajectoryCollection;
//==============================================================================================

class TKeepLaneState : public TVehicleState
{
public:
  TKeepLaneState() {};
  virtual ~TKeepLaneState() override {};

public:
  virtual std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) override;

private:
  void GenerateLaneChangingTrajectories(
    TTrajectoryCollection& aTrajectories,
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    int aTargetLane) const;
};


//==============================================================================================
#endif // KEEPLANESTATE_H
//==============================================================================================
