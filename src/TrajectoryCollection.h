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


public:
  TTrajectoryList::iterator begin() { return mTrajectoryList.begin(); }
  TTrajectoryList::iterator end() { return mTrajectoryList.end(); }


private:
  void UpdateCostForTrajectory(TTrajectory::TTrajectoryPtr apTrajectory);


private:
  double mHorizonTime{10.0};
  double mMaxVelocity{21.0};
  double mVelocityCostFactor{5};
  double mJerkCostFactor{0.5};
  double mTimeCostFactor{0.0};
  double mLaneOffsetFactor{1000.0};
  double mSafetyDistanceFactor{1000};
  std::deque<TTrajectory::TTrajectoryPtr> mTrajectoryList;
  std::vector<Eigen::MatrixXd> mOtherVehicleTrajectories;
};


//==============================================================================================
#endif // TRAJECTORYCOLLECTION_H
//==============================================================================================
