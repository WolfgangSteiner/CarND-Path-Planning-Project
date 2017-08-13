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
#include <iostream>
//==============================================================================================


std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> TChangeLaneState::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  const double kFallbackCostDelta = 40.0;
  const double kCurrentD = aCurrentState(3);
  const double kLaneWidth = NUtils::SLaneWidth();
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);
  const double kTargetD = NUtils::SDForLaneNumber(mTargetLane);
  const double kStartD = NUtils::SDForLaneNumber(mStartLane);
  const double kLaneChangeTime = std::max(4.0, abs(mTargetLane - mStartLane) * 4.0);
  const double kVelocityCorrection =
    std::min(VelocityCorrection(mStartLane), VelocityCorrection(mTargetLane));

  int FallbackLane;
  if (mTargetLane > mStartLane)
  {
    FallbackLane = std::max(mStartLane, kCurrentLane - 1);
  }
  else if (mTargetLane < mStartLane)
  {
    FallbackLane = std::min(mStartLane, kCurrentLane + 1);
  }
  else
  {
    FallbackLane = mTargetLane;
  }


  const double kFallbackD = NUtils::SDForLaneNumber(FallbackLane);


  TTrajectoryCollection LaneChangeTrajectories(mMaxVelocity * kVelocityCorrection, mHorizonTime);
  TTrajectoryCollection FallbackTrajectories(mMaxVelocity * kVelocityCorrection, mHorizonTime);

  LaneChangeTrajectories.SetOtherVehicleTrajectories(
    aSensorFusion.LeadingVehicleTrajectoriesInLane(
      aCurrentState, mStartLane, mCostDeltaT, mHorizonTime));


  if (FallbackLane != mTargetLane)
  {

    FallbackTrajectories.SetOtherVehicleTrajectories(
      aSensorFusion.LeadingVehicleTrajectoriesInLane(
          aCurrentState, mStartLane, mCostDeltaT, mHorizonTime));

    FallbackTrajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
          aCurrentState, FallbackLane, mCostDeltaT, mHorizonTime));
  }


  if (abs(mTargetLane - mStartLane) == 2)
  {
    LaneChangeTrajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 1, mCostDeltaT, mHorizonTime));
  }

  LaneChangeTrajectories.AddOtherVehicleTrajectories(
    aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
      aCurrentState, mTargetLane, mCostDeltaT, mHorizonTime));


  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      LaneChangeTrajectories.AddTrajectory(
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, kLaneChangeTime));


      if (FallbackLane != mTargetLane)
      {
        FallbackTrajectories.AddTrajectory(
            TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kFallbackD, T, kLaneChangeTime));
      }
    }
  }

  auto pMinimumLaneChangeTrajectory = LaneChangeTrajectories.MinimumCostTrajectory();


  std::cout << "===============================================================================" << std::endl;
  std::cout << "Minimum Cost Trajectory:" << std::endl;
  pMinimumLaneChangeTrajectory->PrintCost();


  if (FallbackLane != mTargetLane)
  {
    auto pMinimumFallbackTrajectory = FallbackTrajectories.MinimumCostTrajectory();

    std::cout << "Minimum Fallback Trajectory to lane " << FallbackLane << ":" << std::endl;
    pMinimumFallbackTrajectory->PrintCost();


    if (pMinimumFallbackTrajectory->SafetyDistanceCost() + kFallbackCostDelta < pMinimumLaneChangeTrajectory->SafetyDistanceCost())
    {
      pMinimumLaneChangeTrajectory = pMinimumFallbackTrajectory;
      mTargetLane = FallbackLane;
    }
  }

  std::cout << "===============================================================================" << std::endl;
  std::cout << std::endl << std::endl;


  TVehicleState* pNextState = this;
  if (NUtils::SDistance(kCurrentD, kTargetD) < 0.1)
  {
    pNextState = nullptr;
  }



  return std::make_tuple(pMinimumLaneChangeTrajectory, pNextState);
}


//==============================================================================================
