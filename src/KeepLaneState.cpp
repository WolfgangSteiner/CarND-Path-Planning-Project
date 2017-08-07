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
  std::vector<Eigen::MatrixXd> OtherTrajectories;

  if (LeadingVehicles.size())
  {
    OtherTrajectories.push_back(LeadingVehicles.front().CurrentTrajectory(0.1, mHorizonTime));
  }

  TTrajectoryCollection TrajectoryCollection;
  for (double v = 0; v <= mMaxVelocity; v += 1.0)
  {
    for (double T = 1; T < 10.0; T += 1.0)
    {
      TTrajectory::TTrajectoryPtr ipTrajectory = TTrajectory::SVelocityKeepingTrajectory(aCurrentState, v, aCurrentTime, T);
      UpdateCostForTrajectory(ipTrajectory, OtherTrajectories);
      TrajectoryCollection.AddTrajectory(ipTrajectory);
    }
  }

  TTrajectory::TTrajectoryPtr pMinimumCostTrajectory = TrajectoryCollection.MinimumCostTrajectory();
  const double kMinimumCost = pMinimumCostTrajectory->Cost();

  std::cout << "===============================================================================" << std::endl;
  std::cout << "Minimum Cost Trajectory:" << std::endl;
  pMinimumCostTrajectory->PrintCost();
  std::cout << "===============================================================================" << std::endl;
  std::cout << std::endl << std::endl;

  return pMinimumCostTrajectory;
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
