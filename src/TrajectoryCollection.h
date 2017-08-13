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
  TTrajectoryCollection(double aMaxVelocity, double aHorizonTime);

public:
  void AddTrajectory(TTrajectory::TTrajectoryPtr apTrajectory);
  void SetOtherVehicleTrajectories(const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories);
  void AddOtherVehicleTrajectories(const std::vector<Eigen::MatrixXd>& aOtherVehicleTrajectories);

public:
  using TTrajectoryList = std::deque<TTrajectory::TTrajectoryPtr>;


public:
  TTrajectory::TTrajectoryPtr MinimumCostTrajectory();
  TTrajectory::TTrajectoryPtr MinimumCostLaneChangingTrajectory(double aCurrentD, double aDeltaD);


public:
  TTrajectoryList::iterator begin() { return mTrajectoryList.begin(); }
  TTrajectoryList::iterator end() { return mTrajectoryList.end(); }


private:
  void UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory);


private:
  double mHorizonTime;
  double mMaxVelocity;
  double mVelocityCostFactor;
  double mAccelerationCostFactor;
  double mJerkCostFactor;
  double mTimeCostFactor;
  double mLaneOffsetFactor;
  double mSafetyDistanceFactor;
  std::deque<TTrajectory::TTrajectoryPtr> mTrajectoryList;
  std::vector<Eigen::MatrixXd> mOtherVehicleTrajectories;
};


//==============================================================================================
#endif // TRAJECTORYCOLLECTION_H
//==============================================================================================
