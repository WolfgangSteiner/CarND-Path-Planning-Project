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
  TTrajectory::TTrajectoryPtr pTrajectory;
  TVehicleState* pNextState;

  std::tie(pTrajectory, pNextState) =
    CurrentVehicleState()->Execute(aCurrentState, aCurrentTime, aSensorFusion);

  if (pNextState == nullptr)
  {
    mStateQueue.pop_front();
  }
  else if (pNextState != CurrentVehicleState())
  {
    mStateQueue.push_front(std::shared_ptr<TVehicleState>(pNextState));
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
