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
  const double kCurrentD = aCurrentState(3);
  const double kLaneWidth = NUtils::SLaneWidth();
  const int kCurrentLane = NUtils::SLaneNumberForD(kCurrentD);
  const double kTargetD = NUtils::SDForLaneNumber(mTargetLane);
  const double kStartD = NUtils::SDForLaneNumber(mStartLane);
  const double kLaneChangeTime = abs(mTargetLane - mStartLane) * 4.0;

  std::vector<Eigen::MatrixXd> OtherTrajectories;

  for (TOtherCar& iOtherCar : aSensorFusion.OtherLeadingCarsInLane(aCurrentState))
  {
    OtherTrajectories.push_back(iOtherCar.CurrentTrajectory(0.1, mHorizonTime));
  }

  const double kVelocityCorrection =
    std::min(VelocityCorrection(mStartLane), VelocityCorrection(mTargetLane));

  TTrajectoryCollection Trajectories(mMaxVelocity * kVelocityCorrection, mHorizonTime);

  Trajectories.SetOtherVehicleTrajectories(
    aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
      aCurrentState, mStartLane, mCostDeltaT, mHorizonTime));

  if (abs(mTargetLane - mStartLane) == 2)
  {
    Trajectories.AddOtherVehicleTrajectories(
      aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
        aCurrentState, 1, mCostDeltaT, mHorizonTime));
  }

  Trajectories.AddOtherVehicleTrajectories(
    aSensorFusion.OtherVehicleTrajectoriesInTargetLane(
      aCurrentState, mTargetLane, mCostDeltaT, mHorizonTime));

  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      Trajectories.AddTrajectory(
        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, aCurrentTime, v, kTargetD, T, kLaneChangeTime));

//      Trajectories.AddTrajectory(
//        TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T, kStartD));
    }
  }

  auto pMinimumCostTrajectory = Trajectories.MinimumCostTrajectory();

//  if (pMinimumCostTrajectory->TargetD() == kStartD)
//  {
//    mTargetLane = mStartLane;
//  }

  TVehicleState* pNextState = this;
  if (NUtils::SDistance(kCurrentD, kTargetD) < 0.1)
  {
    pNextState = nullptr;
  }


  std::cout << "===============================================================================" << std::endl;
  std::cout << "Minimum Cost Trajectory:" << std::endl;
  pMinimumCostTrajectory->PrintCost();
  std::cout << "===============================================================================" << std::endl;
  std::cout << std::endl << std::endl;


  return std::make_tuple(pMinimumCostTrajectory, pNextState);
}


//==============================================================================================
