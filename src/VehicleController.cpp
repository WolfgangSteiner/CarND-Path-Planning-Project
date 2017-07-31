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
  mCurrentState = VectorXd::Zero(6);

  QueueTrajectory(21.0, -6.0, 20.0, 10.0);
  QueueTrajectory(21.0, -6.0, 21.0 * 10.0, 10.0);
  QueueTrajectory(21.0, -2.0, 21.0 * 5.0, 5.0);
  QueueTrajectory(21.0, -2.0, 21.0 * 10.0, 10.0);
  QueueTrajectory(21.0, -6.0, 21.0 * 5.0, 5.0);
  QueueTrajectory(21.0, -6.0, 21.0 * 20.0, 20.0);
}

//----------------------------------------------------------------------------------------------

void VehicleController::QueueTrajectory(double aEndVelocity, double aEndD, double aDeltaS, double aDeltaT)
{
  mTrajectoryQueue.push_back(TTrajectoryPtr(new Trajectory(aEndVelocity, aEndD, aDeltaS, aDeltaT)));
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

  if (mpCurrentTrajectory == nullptr)
  {
    mCurrentTime = 0.0;
    mCurrentState << aCarState.s, 0.0, 0.0, aCarState.d, 0.0, 0.0;
  }

  for (int i = kPathSize; i < n; ++i)
  {
    if (mpCurrentTrajectory == nullptr || mpCurrentTrajectory->IsFinished(mCurrentTime))
    {
      mpCurrentTrajectory = mTrajectoryQueue.front();
      mTrajectoryQueue.pop_front();
      mpCurrentTrajectory->Finalize(mCurrentState, mCurrentTime);
    }

    mCurrentState = mpCurrentTrajectory->EvalAt(mCurrentTime);
    const auto p = mWaypoints.getXY_interpolated(mCurrentState(0), mCurrentState(3));
    aNextPathX.push_back(p(0));
    aNextPathY.push_back(p(1));
    mCurrentTime += 0.02;
  }
}


//==============================================================================================
