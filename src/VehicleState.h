//==============================================================================================
#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include "Trajectory.h"
//==============================================================================================
class TSensorFusion;
//==============================================================================================

class TVehicleState
{
public:
  TVehicleState() {}
  virtual ~TVehicleState() {}

public:
  virtual TTrajectory::TTrajectoryPtr Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) = 0;

  virtual TVehicleState* NextVehicleState(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) = 0;

  void UpdateCostForTrajectory(
    TTrajectory::TTrajectoryPtr pTrajectory,
    const std::vector<Eigen::MatrixXd>& aOtherTrajectories);

protected:
  double mHorizonTime{4.0};
  double mMaxVelocity{22};
  double mVelocityCostFactor{5.0};
  double mJerkCostFactor{0.5};
  double mTimeCostFactor{0.0};
  double mSafetyDistanceFactor{1000};
};


//==============================================================================================
#endif //VEHICLESTATE_H
//==============================================================================================