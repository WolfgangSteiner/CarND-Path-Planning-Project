#include "TrajectoryCollection.h"//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "TrajectoryCollection.h"
#include "Utils.h"
//==============================================================================================

//==============================================================================================

TTrajectoryCollection::TTrajectoryCollection(double aMaxVelocity, double aHorizonTime)
: mMaxVelocity(aMaxVelocity)
, mHorizonTime(aHorizonTime)
, mVelocityCostFactor{1.0}
, mAccelerationCostFactor{0.75}
, mJerkCostFactor{0.025}
, mTimeCostFactor{0.0}
, mLaneOffsetFactor{0.5}
, mLaneFactor{0.0}
, mSafetyDistanceFactor{1000}
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
  }

  assert(pMinTrajectory != nullptr);
  return pMinTrajectory;
}



//----------------------------------------------------------------------------------------------

TTrajectory::TTrajectoryPtr TTrajectoryCollection::MinimumCostLaneChangingTrajectory(
  double aCurrentD,
  double aDeltaD)
{
  assert(mTrajectoryList.size());
  double MinCost = 1.0e12;
  TTrajectory::TTrajectoryPtr pMinTrajectory;

  for (TTrajectory::TTrajectoryPtr ipTrajectory : mTrajectoryList)
  {
    if (ipTrajectory->Cost() < MinCost
        && abs(NUtils::SDistance(ipTrajectory->TargetD(), aCurrentD) - aDeltaD) < 0.1)
    {
      MinCost = ipTrajectory->Cost();
      pMinTrajectory = ipTrajectory;
    }
  }

  return pMinTrajectory;
}



//----------------------------------------------------------------------------------------------

void TTrajectoryCollection::UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  const double kAccelerationCost = mAccelerationCostFactor * apTrajectory->AccelerationCost(mHorizonTime);
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
    const double kSafetyDistanceCost = apTrajectory->SafetyDistanceCost(iOtherTrajectory, mHorizonTime);
    MaxSafetyDistanceCost = std::max(MaxSafetyDistanceCost, kSafetyDistanceCost);
  }

  apTrajectory->SetVelocityCost(VelocityCost);
  apTrajectory->SetAccelerationCost(kAccelerationCost);
  apTrajectory->SetJerkCost(kJerkCost);
  apTrajectory->SetTimeCost(kTimeCost);
  apTrajectory->SetSafetyDistanceCost(mSafetyDistanceFactor * MaxSafetyDistanceCost);
  apTrajectory->SetLaneOffsetCost(mLaneOffsetFactor * apTrajectory->LaneOffsetCost(kDuration));
  apTrajectory->SetLaneCost(mLaneFactor * apTrajectory->TargetLaneCost());

#if 0
  apTrajectory->PrintCost();
#endif

}

//==============================================================================================
