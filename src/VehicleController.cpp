//==============================================================================================
#include "VehicleController.h"
#include "CarState.h"
#include "Trajectory.h"
//==============================================================================================
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using std::vector;
//==============================================================================================

VehicleController::VehicleController()
{
  mDesiredState = VectorXd(6);
  mDesiredState << 0.0, 10.0, 0.0, -6.0, 0.0, 0.0;
}


//----------------------------------------------------------------------------------------------

void VehicleController::UpdateTrajectory(
    const std::vector<double>& aPreviousPathX,
    const std::vector<double>& aPreviousPathY,
    const CarState& aCarState,
    std::vector<double>& aNextPathX,
    std::vector<double>& aNextPathY)
{
  const double delta_s = 0.4;
  const int n = int(mTimeHorizon * 1000) / 20;

  const int kPathSize = aPreviousPathX.size();
  for (int i = 0; i < kPathSize; ++i) {
    aNextPathX.push_back(aPreviousPathX[i]);
    aNextPathY.push_back(aPreviousPathY[i]);
  }

  double s = aCarState.s;
  if (mpCurrentTrajectory == nullptr) {
    VectorXd start_state = VectorXd(6);
    start_state << aCarState.s, 0.0, 0.0, -6.0, 0.0, 0.0;

    VectorXd end_state = VectorXd(6);
    end_state << aCarState.s + 100, 10.0, 0.0, -10.0, 0.0, 0.0;

    mpCurrentTrajectory = TTrajectoryPtr(new Trajectory(start_state, end_state, 0.0, 20.0));
    mCurrentTime = 0.0;
  }

  for (int i = kPathSize; i < n; ++i) {
    const VectorXd state = mpCurrentTrajectory->EvalAt(mCurrentTime);
    const auto p = mWaypoints.getXY_interpolated(state(0), state(3));
    aNextPathX.push_back(p(0));
    aNextPathY.push_back(p(1));
    mCurrentTime += 0.02;
  }
}


//==============================================================================================
