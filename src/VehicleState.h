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
  double mHorizonTime{10.0};
  double mMaxVelocity{21.5};
  double mVelocityCostFactor{0.5};
  double mJerkCostFactor{0.5};
  double mTimeCostFactor{0.0};
  double mSafetyDistanceFactor{1000};
  double mCostDeltaT{0.1};
};


//==============================================================================================
#endif //VEHICLESTATE_H
//==============================================================================================