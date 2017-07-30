//==============================================================================================
#include "VehicleController.h"
#include "CarState.h"
//==============================================================================================

VehicleController::VehicleController()
{}


void VehicleController::UpdateTrajectory(
    const std::vector<double>& aPreviousPathX,
    const std::vector<double>& aPreviousPathY,
    const CarState& aCarState,
    std::vector<double>& aNextPathX,
    std::vector<double>& aNextPathY)
{
  const double delta_s = 0.4;
  const int n = 2000 / 20;

  const int kPathSize = aPreviousPathX.size();
  for (int i = 0; i < kPathSize; ++i)
  {
    aNextPathX.push_back(aPreviousPathX[i]);
    aNextPathY.push_back(aPreviousPathY[i]);
  }

  double s = aCarState.s;

  if (kPathSize)
  {
    s = last_s;
  }

  for (int i = kPathSize; i < n; ++i)
  {
      s += delta_s;
      const auto p = mWaypoints.getXY_interpolated(s, -6.0);
      last_s = s;
      //cout << s << ", " << p(0) << ", " << p(1) << endl;
      aNextPathX.push_back(p(0));
      aNextPathY.push_back(p(1));
  }
}


//==============================================================================================
