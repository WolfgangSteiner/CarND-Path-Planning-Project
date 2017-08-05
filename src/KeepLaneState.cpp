//==============================================================================================
// Created by Wolfgang Steiner on 01.08.17.
//==============================================================================================
#include "KeepLaneState.h"
#include "SensorFusion.h"
#include "Utils.h"
#include "Trajectory.h"
#include "TrajectoryCollection.h"
//==============================================================================================
#include <cmath>
#include <iostream>
#include <tuple>
//==============================================================================================

TTrajectory::TTrajectoryPtr TKeepLaneState::Execute(
  const Eigen::VectorXd& aCurrentState,
  double aCurrentTime,
  const TSensorFusion& aSensorFusion)
{
  auto LeadingVehicles = aSensorFusion.OtherLeadingCarsInLane(aCurrentState);
  const double kCurrentS = aCurrentState(0);
  const double kCurrentSpeed = aCurrentState(1);
  const double kCurrentSafetyDistance = kCurrentSpeed * 3.6 / 2;

  TOtherCar* pLeadingVehicle = nullptr;
  Eigen::MatrixXd LeadingVehicleTrajectory;

  if (LeadingVehicles.size())
  {
    pLeadingVehicle = &LeadingVehicles.front();
    LeadingVehicleTrajectory = pLeadingVehicle->CurrentTrajectory(0.1, mHorizonTime);
  }

  TTrajectoryCollection TrajectoryCollection;
  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      TTrajectory::TTrajectoryPtr ipTrajectory = TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T);
      const double kJerkCost = mJerkCostFactor * ipTrajectory->JerkCost();
      const double kTimeCost = mTimeCostFactor * T;
      const double kMinVelocity = ipTrajectory->MinVelocity();
      const double kMaxVelocity = ipTrajectory->MaxVelocity();
      double VelocityCost = mVelocityCostFactor * pow(mMaxVelocity - v, 2);

      if (kMinVelocity < 0 || kMaxVelocity > mMaxVelocity)
      {
        VelocityCost += 1000;
      }

//      double AccelerationCost = std::exp()

      ipTrajectory->AddCost(VelocityCost);
      ipTrajectory->AddCost(kJerkCost);
      ipTrajectory->AddCost(kTimeCost);

      double SafetyDistanceCost = 0.0;
      double MinDistToLeadingVehicle = 1000;

      if (pLeadingVehicle)
      {
        double Min_time;
        const double kDuration = mHorizonTime;
        const double kDeltaT = 0.1;
        std::tie(MinDistToLeadingVehicle, Min_time) =
          ipTrajectory->MinDistanceToTrajectory(LeadingVehicleTrajectory, kDeltaT, kDuration);
        SafetyDistanceCost = ipTrajectory->SafetyDistanceCost(LeadingVehicleTrajectory, kDeltaT, kDuration);
        SafetyDistanceCost *= mSafetyDistanceFactor;
        ipTrajectory->AddCost(SafetyDistanceCost);
      }

      std::cout
           << "s: "        << aCurrentState(0)
           << " v: "       << v
           << " T: "       << T
           << " Vc: "      << VelocityCost
           << " Jc: "      << kJerkCost
           << " MinDist: " << MinDistToLeadingVehicle
           << " Dc: "      << SafetyDistanceCost
           << " v_min: "   << kMinVelocity
           << " v_max "    << kMaxVelocity
           << " cost: "    << ipTrajectory->Cost()
           << std::endl;

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
