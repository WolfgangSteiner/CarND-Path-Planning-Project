//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#include "TrajectoryCollection.h"
//==============================================================================================

//==============================================================================================
void TTrajectoryCollection::AddTrajectory(TTrajectory::TTrajectoryPtr apTrajectory)
{
  mTrajectoryList.push_back(apTrajectory);
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


//==============================================================================================
