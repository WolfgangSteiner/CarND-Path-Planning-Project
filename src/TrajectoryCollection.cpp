#include "TrajectoryCollection.h"//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "TrajectoryCollection.h"
//==============================================================================================

//==============================================================================================

TTrajectoryCollection::TTrajectoryCollection(double aMaxVelocity, double aHorizonTime)
: mMaxVelocity(aMaxVelocity)
, mHorizonTime(aHorizonTime)
, mVelocityCostFactor{0.5}
, mJerkCostFactor{5}
, mTimeCostFactor{0.0}
, mLaneOffsetFactor{0.5}
, mSafetyDistanceFactor{100}
{}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::AddTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  mTrajectoryList.push_back(apTrajectory);
  UpdateCostForTrajectory(apTrajectory);
}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::SetOtherVehicleTrajectories(
  const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories)
{
  mOtherVehicleTrajectories = aOtherVehicleTrajectories;
}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::AddOtherVehicleTrajectories(
  const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories)
{
  mOtherVehicleTrajectories.insert(
    mOtherVehicleTrajectories.end(),
    aOtherVehicleTrajectories.begin(),
    aOtherVehicleTrajectories.end());
}


//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectoryCollection::MinimumCostTrajectory()
{
  assert(mTrajectoryList.size());
  double MinCost = 1.0e12;
  TTrajectory::TTrajectoryPtr pMinTrajectory;

  for (TTrajectory::TTrajectoryPtr ipTrajectory : mTrajectoryList)
  {
    if (ipTrajectory->Cost() < MinCost)
    {
      MinCost = ipTrajectory->Cost();
      pMinTrajectory = ipTrajectory;
    }
    else if (ipTrajectory->Cost() > 1.0e12)
    {
      ipTrajectory->PrintCost();
    }
  }

  assert(pMinTrajectory != nullptr);
  return pMinTrajectory;
}


//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  const double kJerkCost = mJerkCostFactor * apTrajectory->JerkCost(mHorizonTime);
  const double kTimeCost = mTimeCostFactor * apTrajectory->DurationS();
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
  apTrajectory->SetLaneOffsetCost(mLaneOffsetFactor * apTrajectory->LaneOffsetCost(kDuration));

#if 0
  apTrajectory->PrintCost();
#endif

}

//==============================================================================================
