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

protected:
  double mHorizonTime{1.0};
};


//==============================================================================================
#endif //VEHICLESTATE_H
//==============================================================================================