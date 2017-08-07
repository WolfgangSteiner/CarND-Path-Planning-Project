//==============================================================================================
#include "VehicleState.h"
//==============================================================================================
#include <iostream>
//==============================================================================================

void TVehicleState::UpdateCostForTrajectory(
  TTrajectory::TTrajectoryPtr pTrajectory,
  const std::vector<Eigen::MatrixXd>& aOtherTrajectories)
{
  const double kJerkCost = mJerkCostFactor * pTrajectory->JerkCost(mHorizonTime);
  const double kTimeCost = mTimeCostFactor * pTrajectory->Duration();
  const double kMinVelocity = pTrajectory->MinVelocity();
  const double kMaxVelocity = pTrajectory->MaxVelocity();
  double VelocityCost = mVelocityCostFactor * pTrajectory->VelocityCost(mMaxVelocity, mHorizonTime);
  //assert(VelocityCost <= pow(mMaxVelocity, 2) * mVelocityCostFactor);


  if (kMinVelocity < 0 || kMaxVelocity > mMaxVelocity)
  {
    VelocityCost += 1000;
  }

//  pTrajectory->AddCost(VelocityCost);
//  pTrajectory->AddCost(kJerkCost);
//  pTrajectory->AddCost(kTimeCost);

  const double kDuration = mHorizonTime;
  const double kDeltaT = 0.1;
  double MaxSafetyDistanceCost = 0;

  for (const auto& iOtherTrajectory : aOtherTrajectories)
  {
    const double kSafetyDistanceCost = pTrajectory->SafetyDistanceCost(iOtherTrajectory, kDuration);
    MaxSafetyDistanceCost = std::max(MaxSafetyDistanceCost, kSafetyDistanceCost);
  }

  pTrajectory->SetVelocityCost(VelocityCost);
  pTrajectory->SetJerkCost(kJerkCost);
  pTrajectory->SetTimeCost(kTimeCost);
  pTrajectory->SetSafetyDistanceCost(mSafetyDistanceFactor * MaxSafetyDistanceCost);

#if 1
  pTrajectory->PrintCost();
#endif

}


//==============================================================================================

