#include "TrajectoryCollection.h"//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "TrajectoryCollection.h"
//==============================================================================================

//==============================================================================================

TTrajectoryCollection::TTrajectoryCollection(
  double aMaxVelocity,
  double aHorizonTime,
  const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories)
: mMaxVelocity(aMaxVelocity)
, mHorizonTime(aHorizonTime)
, mOtherVehicleTrajectories(aOtherVehicleTrajectories)
{}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::AddTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  mTrajectoryList.push_back(apTrajectory);
  UpdateCostForTrajectory(apTrajectory);
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectoryCollection::MinimumCostTrajectory()
{
  double MinCost = 1e9;
  TTrajectory::TTrajectoryPtr pMinTrajectory;

  for (TTrajectory::TTrajectoryPtr ipTrajectory : mTrajectoryList)
  {
    if (ipTrajectory->Cost() < MinCost)
    {
      MinCost = ipTrajectory->Cost();
      pMinTrajectory = ipTrajectory;
    }
  }

  assert(pMinTrajectory != nullptr);
  return pMinTrajectory;
}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  const double kJerkCost = mJerkCostFactor * apTrajectory->JerkCost(mHorizonTime);
  const double kTimeCost = mTimeCostFactor * apTrajectory->Duration();
  const double kMinVelocity = apTrajectory->MinVelocity();
  const double kMaxVelocity = apTrajectory->MaxVelocity();
  double VelocityCost = mVelocityCostFactor * apTrajectory->VelocityCost(mMaxVelocity, mHorizonTime);
  //assert(VelocityCost <= pow(mMaxVelocity, 2) * mVelocityCostFactor);


  if (kMinVelocity < 0 || kMaxVelocity > mMaxVelocity)
  {
    VelocityCost += 1000;
  }

  const double kDuration = mHorizonTime;
  const double kDeltaT = 0.1;
  double MaxSafetyDistanceCost = 0;

  for (const auto& iOtherTrajectory : mOtherVehicleTrajectories)
  {
    const double kSafetyDistanceCost = apTrajectory->SafetyDistanceCost(iOtherTrajectory, kDuration);
    MaxSafetyDistanceCost = std::max(MaxSafetyDistanceCost, kSafetyDistanceCost);
  }

  apTrajectory->SetVelocityCost(VelocityCost);
  apTrajectory->SetJerkCost(kJerkCost);
  apTrajectory->SetTimeCost(kTimeCost);
  apTrajectory->SetSafetyDistanceCost(mSafetyDistanceFactor * MaxSafetyDistanceCost);

#if 0
  apTrajectory->PrintCost();
#endif

}

//==============================================================================================
