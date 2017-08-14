//==============================================================================================
#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H
//==============================================================================================
#include "Waypoints.h"
#include "Trajectory.h"
#include "SensorFusion.h"
#include "StateMachine.h"
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include <deque>
#include <memory>
//==============================================================================================
struct CarState;
//==============================================================================================

class TVehicleController
{
public:
  TVehicleController();

public:
  void UpdateTrajectory(
      const std::vector<double>& aPreviousPathX,
      const std::vector<double>& aPreviousPathY,
      const CarState& aCarState,
      std::vector<double>& aNextPathX,
      std::vector<double>& aNextPathY,
      const std::vector<std::vector<double>>& aSensorFusion);

//  void QueueTrajectory(TTrajectoryPtr apTrajectory);
//  void QueueTrajectory(double aEndVelocity, double aEndD, double aDeltaS, double aDeltaT);


private:
  Eigen::VectorXd mCurrentState;
  TTrajectory::TTrajectoryPtr mpCurrentTrajectory;
  TStateMachine mStateMachine;
  TWaypoints mWaypoints;
  TSensorFusion mSensorFusion;

  double mCurrentTime{0.0};
  double mPredictionHorizon{0.5};
  double mDisplayHorizon{2.0};
  bool mIsInitialized{false};
};


//==============================================================================================
#endif // VEHICLECONTROLLER_H
//==============================================================================================
