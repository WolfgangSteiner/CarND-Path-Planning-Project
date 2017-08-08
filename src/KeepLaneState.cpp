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
  const double kLaneChangeTime = 4.0;

  TTrajectoryCollection Trajectories(mMaxVelocity * VelocityCorrection(kCurrentLane), mHorizonTime);
  std::vector<Eigen::MatrixXd> LeadingVehicleTrajectories =
    aSensorFusion.LeadingVehicleTrajectoriesInLane(aCurrentState, mCostDeltaT, mHorizonTime);

  // Trajectories that stay in the current lane:
  Trajectories.SetOtherVehicleTrajectories(LeadingVehicleTrajectories);
  const double T_max = 10.0;

  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < T_max; T += 1.0)
    {
      Trajectories.AddTrajectory(
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kCurrentD, T, 0.0));
    }
  }

  // Trajectories that change lane to right:


  if (kCurrentLane >= 1)
  {
    const int kTargetLane = kCurrentLane - 1;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, kTargetLane, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < T_max; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, kLaneChangeTime));
      }
    }
  }

  if (kCurrentLane == 2)
  {
    const int kTargetLane = 0;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 0, mCostDeltaT, mHorizonTime));

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 1, mCostDeltaT, mHorizonTime));


    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < T_max; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, 2.0 * kLaneChangeTime));
      }
    }
  }



  if (kCurrentLane <= 1)
  {
    const int kTargetLane = kCurrentLane + 1;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.SetOtherVehicleTrajectories(LeadingVehicleTrajectories);
    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, kTargetLane, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < T_max; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, kLaneChangeTime));
      }
    }
  }

  if (kCurrentLane == 0)
  {
    const int kTargetLane = 2;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.SetOtherVehicleTrajectories(LeadingVehicleTrajectories);

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 1, mCostDeltaT, mHorizonTime));

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 2, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double T = 1; T < T_max; T += 1.0)
      {
        Trajectories.AddTrajectory(
          TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, 2.0 * kLaneChangeTime));
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
    const int kTargetLane = NUtils::SLaneNumberForD(kTargetD);
    pNextState = new TChangeLaneState(kCurrentLane, kTargetLane);
  }

  return std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*>(pMinimumCostTrajectory, pNextState);
}


//==============================================================================================
