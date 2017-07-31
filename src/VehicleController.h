//==============================================================================================
#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
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
void UpdateTrajectory(
    const std::vector<double>& aPreviousPathX,
    const std::vector<double>& aPreviousPathY,
    const CarState& aCarState,
    std::vector<double>& aNextPathX,
    std::vector<double>& aNextPathY);

private:
  using TTrajectoryPtr = std::shared_ptr<Trajectory>;

private:
  std::vector<Eigen::VectorXd> mCurrentTrajectory;
  std::vector<Eigen::VectorXd> mPreviousTrajectory;
  Eigen::VectorXd mCurrentState;
  Eigen::VectorXd mDesiredState;

  TTrajectoryPtr mpCurrentTrajectory;

  Waypoints mWaypoints;
  double mLastS{0.0};
  double mCurrentTime{0.0};
  double mTimeHorizon = 0.5;
};


//==============================================================================================
#endif // VEHICLECONTROLLER_H
//==============================================================================================
