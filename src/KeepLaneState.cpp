//==============================================================================================
// Created by Wolfgang Steiner on 01.08.17.
//==============================================================================================
#include "ChangeLaneState.h"
#include "KeepLaneState.h"
#include "SensorFusion.h"
#include "Trajectory.h"
#include "TrajectoryCollection.h"
#include "Utils.h"
//==============================================================================================
#include <cmath>
#include <iostream>
#include <tuple>
//==============================================================================================

std::tuple<TTrajectory::TTrajectoryPtr, TVehicleState*> TKeepLaneState::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  const double kCurrentD = aCurrentState(3);
  const double kLaneWidth = NUtils::SLaneWidth();
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);
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
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T, kCurrentD));
    }
  }

  if (kCurrentLane >= 1)
  {
    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < 10.0; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T, kCurrentD - kLaneWidth));
      }
    }
  }

  if (kCurrentLane <= 1)
  {
    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < 10.0; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T, kCurrentD + kLaneWidth));
      }
    }
  }

  TTrajectory::TTrajectoryPtr pMinimumCostTrajectory = Trajectories.MinimumCostTrajectory();
  const double kMinimumCost = pMinimumCostTrajectory->Cost();

  std::cout << "===============================================================================" << std::endl;
  std::cout << "Minimum Cost Trajectory:" << std::endl;
  pMinimumCostTrajectory->PrintCost();
  std::cout << "===============================================================================" << std::endl;
  std::cout << std::endl << std::endl;

  TVehicleState* pNextState = this;
  const double kTargetD = pMinimumCostTrajectory->TargetD();

  if (kTargetD != kCurrentD)
  {
    pNextState = new TChangeLaneState(NUtils::SLaneNumberForD(kTargetD));
  }

  return std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*>(pMinimumCostTrajectory, pNextState);
}


//==============================================================================================
