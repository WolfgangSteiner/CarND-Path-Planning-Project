//==============================================================================================
#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include "Trajectory.h"
//==============================================================================================
class TSensorFusion;
//==============================================================================================
#include <tuple>
//==============================================================================================

class TVehicleState
{
public:
  TVehicleState() {}
  virtual ~TVehicleState() {}

public:
  virtual std::tuple<TTrajectory::TTrajectoryPtr,TVehicleState*> Execute(
    const Eigen::VectorXd& aCurrentState,
    double aCurrentTime,
    const TSensorFusion& aSensorFusion) = 0;

protected:
  double VelocityCorrection(int aLaneNumber) const;

protected:
  double mHorizonTime{4.0};
  double mMaxVelocity{49.0 * 0.447};
  double mCostDeltaT{0.1};
};


//==============================================================================================
#endif //VEHICLESTATE_H
//==============================================================================================
