//==============================================================================================
#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
#include "Waypoints.h"
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
  std::vector<Eigen::VectorXd> mCurrentTrajectory;
  Eigen::VectorXd mCurrentState;
  Waypoints mWaypoints;
  double last_s{0.0};
};


//==============================================================================================
#endif // VEHICLECONTROLLER_H
//==============================================================================================
