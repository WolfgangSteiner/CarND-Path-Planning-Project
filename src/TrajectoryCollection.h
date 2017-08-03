//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef TRAJECTORYCOLLECTION_H
#define TRAJECTORYCOLLECTION_H
//==============================================================================================
#include "Trajectory.h"
//==============================================================================================
#include <deque>
//==============================================================================================

class TTrajectoryCollection
{
public:
  void AddTrajectory(TTrajectory::TTrajectoryPtr apTrajectory);

public:
  using TTrajectoryList = std::deque<TTrajectory::TTrajectoryPtr>;

public:
  TTrajectory::TTrajectoryPtr MinimumCostTrajectory();


public:
  TTrajectoryList::iterator begin() { return mTrajectoryList.begin(); }
  TTrajectoryList::iterator end() { return mTrajectoryList.end(); }

private:
  std::deque<TTrajectory::TTrajectoryPtr> mTrajectoryList;
};


//==============================================================================================
#endif // TRAJECTORYCOLLECTION_H
//==============================================================================================
