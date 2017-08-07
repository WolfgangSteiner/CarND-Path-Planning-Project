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
  TTrajectoryCollection(
    double aMaxVelocity,
    double aHorizonTime,
    const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories);

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
  void UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory);


private:
  double mHorizonTime{10.0};
  double mMaxVelocity{22};
  double mVelocityCostFactor{0.5};
  double mJerkCostFactor{0.5};
  double mTimeCostFactor{0.0};
  double mSafetyDistanceFactor{1000};
  std::deque<TTrajectory::TTrajectoryPtr> mTrajectoryList;
  std::vector<Eigen::MatrixXd> mOtherVehicleTrajectories;
};


//==============================================================================================
#endif // TRAJECTORYCOLLECTION_H
//==============================================================================================
