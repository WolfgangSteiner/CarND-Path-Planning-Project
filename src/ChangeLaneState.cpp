//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "ChangeLaneState.h"
#include "Utils.h"
#include "SensorFusion.h"
#include "Trajectory.h"
#include "TrajectoryCollection.h"
//==============================================================================================
#include <tuple>
//==============================================================================================


std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> TChangeLaneState::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  const double kCurrentD = aCurrentState(3);
  const double kLaneWidth = NUtils::SLaneWidth();
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);
  const double kTargetD = NUtils::SDForLaneNumber(mTargetLane);

  std::vector<Eigen::MatrixXd> OtherTrajectories;

  for (TOtherCar& iOtherCar : aSensorFusion.OtherNearbyCars(aCurrentState))
  {
    OtherTrajectories.push_back(iOtherCar.CurrentTrajectory(0.1, mHorizonTime));
  }

  TTrajectoryCollection Trajectories(mMaxVelocity, mHorizonTime, OtherTrajectories);

  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      Trajectories.AddTrajectory(
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T, kTargetD));
    }
  }

  TVehicleState* pNextState = this;
  if (NUtils::SDistance(kCurrentD, kTargetD) < 0.1)
  {
    pNextState = nullptr;
  }

  auto pMinimumCostTrajectory = Trajectories.MinimumCostTrajectory();
  return std::make_tuple(pMinimumCostTrajectory, pNextState);
}


//==============================================================================================
