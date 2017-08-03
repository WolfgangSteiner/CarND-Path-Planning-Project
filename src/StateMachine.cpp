//==============================================================================================
#include "StateMachine.h"
//==============================================================================================
#include <assert.h>
//==============================================================================================

TStateMachine::TStateMachine(TVehicleState* apInitialState)
{
  mStateQueue.push_back(std::shared_ptr<TVehicleState>(apInitialState));
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TStateMachine::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  TTrajectory::TTrajectoryPtr pTrajectory = CurrentVehicleState()->Execute(aCurrentState, aCurrentTime, aSensorFusion);
  TVehicleState* pNextVehicleState = CurrentVehicleState()->NextVehicleState(aCurrentState, aCurrentTime, aSensorFusion);

  if (pNextVehicleState == nullptr)
  {
    mStateQueue.pop_front();
  }
  else if (pNextVehicleState != CurrentVehicleState())
  {
    mStateQueue.push_front(std::shared_ptr<TVehicleState>(pNextVehicleState));
  }

  return pTrajectory;
}

//----------------------------------------------------------------------------------------------

TVehicleState* TStateMachine::CurrentVehicleState()
{
  assert(mStateQueue.size());
  return mStateQueue.front().get();
}


//==============================================================================================
