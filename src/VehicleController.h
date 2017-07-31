//==============================================================================================
#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include <deque>
#include <memory>
#include "Waypoints.h"
#include "Trajectory.h"
//==============================================================================================
struct CarState;
//==============================================================================================

class VehicleController
{
public:
  VehicleController();


public:
  using TTrajectoryPtr = std::shared_ptr<Trajectory>;


public:
  void UpdateTrajectory(
      const std::vector<double>& aPreviousPathX,
      const std::vector<double>& aPreviousPathY,
      const CarState& aCarState,
      std::vector<double>& aNextPathX,
      std::vector<double>& aNextPathY);

  void QueueTrajectory(TTrajectoryPtr apTrajectory);
  void QueueTrajectory(double aEndVelocity, double aEndD, double aDeltaS, double aDeltaT);


private:
  std::vector<Eigen::VectorXd> mCurrentTrajectory;
  std::vector<Eigen::VectorXd> mPreviousTrajectory;
  Eigen::VectorXd mCurrentState;
  Eigen::VectorXd mDesiredState;

  TTrajectoryPtr mpCurrentTrajectory;
  std::deque<TTrajectoryPtr> mTrajectoryQueue;

  Waypoints mWaypoints;
  double mLastS{0.0};
  double mCurrentTime{0.0};
  double mTimeHorizon = 0.5;
};


//==============================================================================================
#endif // VEHICLECONTROLLER_H
//==============================================================================================
