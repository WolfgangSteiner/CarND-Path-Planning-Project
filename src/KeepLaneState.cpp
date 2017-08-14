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
  const int kCurrentLane = NUtils::SLaneNumberForD(aCurrentState(3));
  const double kCurrentD = NUtils::SDForLaneNumber(kCurrentLane);
  const double kLaneWidth = NUtils::SLaneWidth();
  const double kLaneChangeTime = 4.0;

  TTrajectoryCollection Trajectories(mMaxVelocity * VelocityCorrection(kCurrentLane), mHorizonTime);
  std::vector<Eigen::MatrixXd> ThisLaneVehicleTrajectories =
    aSensorFusion.LeadingVehicleTrajectoriesInLane(aCurrentState, kCurrentLane, mCostDeltaT, mHorizonTime);

  // Trajectories that stay in the current lane:
  Trajectories.SetOtherVehicleTrajectories(ThisLaneVehicleTrajectories);

  const double Ts_max = 8.0;
  const double Ts_min = 2.0;
  const double Ts_delta = 1.0;
  const double Td_max = 4.0;
  const double Td_min = 2.0;
  const double Td_delta = 1.0;

  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double Ts = Ts_min; Ts < Ts_max; Ts += Ts_delta)
    {
      Trajectories.AddTrajectory(
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kCurrentD, Ts, Ts));
    }
  }

  // Trajectories that change lane to right:


  if (kCurrentLane >= 1)
  {
    const int kTargetLane = kCurrentLane - 1;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.SetOtherVehicleTrajectories(ThisLaneVehicleTrajectories);

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, kTargetLane, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double Ts = Ts_min; Ts < Ts_max; Ts += Ts_delta)
      {
        for (double Td = Td_min; Td < Td_max; Td += Td_delta)
        {
          Trajectories.AddTrajectory(
            TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, Ts, Td));
        }
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


    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double Ts = Ts_min; Ts < Ts_max; Ts += Ts_delta)
      {
        for (double Td = Td_min; Td < Td_max; Td += Td_delta)
        {
          Trajectories.AddTrajectory(
            TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, Ts, Td));
        }
      }
    }
  }



  if (kCurrentLane <= 1)
  {
    const int kTargetLane = kCurrentLane + 1;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.SetOtherVehicleTrajectories(ThisLaneVehicleTrajectories);
    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, kTargetLane, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double Ts = Ts_min; Ts < Ts_max; Ts += Ts_delta)
      {
        for (double Td = Td_min; Td < Td_max; Td += Td_delta)
        {
          Trajectories.AddTrajectory(
            TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, Ts, Td));
        }
      }
    }
  }

  if (kCurrentLane == 0)
  {
    const int kTargetLane = 2;
    const double kTargetD = NUtils::SDForLaneNumber(kTargetLane);

    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 2, mCostDeltaT, mHorizonTime));

    for (double v = 0; v <= mMaxVelocity; v += 1.0)
    {
      for (double Ts = Ts_min; Ts < Ts_max; Ts += Ts_delta)
      {
        for (double Td = Td_min; Td < Td_max; Td += Td_delta)
        {
          Trajectories.AddTrajectory(
            TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, Ts, Td));
        }
      }
    }
  }


  TTrajectory::TTrajectoryPtr pMinimumCostTrajectory = Trajectories.MinimumCostTrajectory();
  const double kMinimumCost = pMinimumCostTrajectory->Cost();

  auto pMinimumCostLaneChangingTrajectory1 = Trajectories.MinimumCostLaneChangingTrajectory(kCurrentD, NUtils::SLaneWidth());
  auto pMinimumCostLaneChangingTrajectory2 = Trajectories.MinimumCostLaneChangingTrajectory(kCurrentD, 2.0 * NUtils::SLaneWidth());

  std::cout << "===============================================================================" << std::endl;
  std::cout << "Minimum Cost Trajectory:" << std::endl;
  pMinimumCostTrajectory->PrintCost();
  if (pMinimumCostLaneChangingTrajectory1 != nullptr)
  {
    std::cout << "Minimum Cost Lane Changing Trajectory 1:" << std::endl;
    pMinimumCostLaneChangingTrajectory1->PrintCost();
  }
  if (pMinimumCostLaneChangingTrajectory2 != nullptr)
  {
    std::cout << "Minimum Cost Lane Changing Trajectory 2:" << std::endl;
    pMinimumCostLaneChangingTrajectory2->PrintCost();
  }
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
