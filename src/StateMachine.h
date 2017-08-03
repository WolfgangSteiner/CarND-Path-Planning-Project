//==============================================================================================
#ifndef STATEMACHINE_H
#define STATEMACHINE_H
//==============================================================================================
#include "VehicleState.h"
#include "Trajectory.h"
//==============================================================================================
#include <deque>
#include <memory>
//==============================================================================================
class TSensorFusion;
//==============================================================================================

class TStateMachine
{
public:
  TStateMachine(TVehicleState* apInitialState);

public:
  TTrajectory::TTrajectoryPtr Execute(
    const Eigen::VectorXd& mCurrentState,
    double mCurrentTime,
    const TSensorFusion& aSensorFusion);

  TVehicleState* CurrentVehicleState();

private:
  std::deque<std::shared_ptr<TVehicleState>> mStateQueue;
};

//==============================================================================================
#endif // STATEMACHINE_H
//==============================================================================================