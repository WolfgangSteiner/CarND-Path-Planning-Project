//==============================================================================================
// Created by Wolfgang Steiner on 01.08.17.
//==============================================================================================
#include "KeepLaneState.h"
#include "SensorFusion.h"
#include "Utils.h"
#include "Trajectory.h"
#include "TrajectoryCollection.h"
#include <cmath>
//==============================================================================================

//==============================================================================================

TTrajectory::TTrajectoryPtr TKeepLaneState::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  const auto& LeadingVehicles = aSensorFusion.OtherLeadingCarsInLane(aCurrentState);
  const double kCurrentS = aCurrentState(0);
  const double kCurrentSpeed = aCurrentState(1);
  const double kCurrentSafetyDistance = kCurrentSpeed * 3.6 / 2;

  TTrajectory::TTrajectoryPtr pLeadingVehicleTrajectory;

  if (LeadingVehicles.size())
  {
    const TOtherCar& kLeadingVehicle = LeadingVehicles.front();
    pLeadingVehicleTrajectory = kLeadingVehicle.CurrentTrajectory(aCurrentTime, mHorizonTime);
  }

  TTrajectoryCollection TrajectoryCollection;
  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      TTrajectory::TTrajectoryPtr ipTrajectory = TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T);
      const double kVelocityCost = mVelocityCostFactor * pow(mMaxVelocity - v, 2);
      const double kJerkCost = mJerkCostFactor * ipTrajectory->JerkCost();
      const double kTimeCost = mTimeCostFactor * T;
      const double kMinVelocity = ipTrajectory->MinVelocity();
      const double kMaxVelocity = ipTrajectory->MaxVelocity();

      if (kMinVelocity < 0 || kMaxVelocity > mMaxVelocity)
      {
        continue;
      }

      ipTrajectory->AddCost(kVelocityCost);
      ipTrajectory->AddCost(kJerkCost);
      ipTrajectory->AddCost(kTimeCost);

      if (pLeadingVehicleTrajectory)
      {
        const double kSafetyDistanceCost = ipTrajectory->SafetyDistanceCost(pLeadingVehicleTrajectory);
        ipTrajectory->AddCost(kSafetyDistanceCost);
      }

      TrajectoryCollection.AddTrajectory(ipTrajectory);
    }
  }

  return TrajectoryCollection.MinimumCostTrajectory();
}

//----------------------------------------------------------------------------------------------

TVehicleState* TKeepLaneState::NextVehicleState(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  return this;
}


//==============================================================================================
