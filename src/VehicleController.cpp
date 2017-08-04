//==============================================================================================
#include "VehicleController.h"
#include "CarState.h"
#include "KeepLaneState.h"
#include "SensorFusion.h"
//==============================================================================================
#include <iostream>
//==============================================================================================
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using std::vector;
//==============================================================================================

TVehicleController::TVehicleController()
: mStateMachine(new TKeepLaneState())
{
  mCurrentState = VectorXd::Zero(6);
}

//----------------------------------------------------------------------------------------------
//
//void TVehicleController::QueueTrajectory(double aEndVelocity, double aEndD, double aDeltaS, double aDeltaT)
//{
//  mTrajectoryQueue.push_back(TTrajectoryPtr(new TTrajectory(aEndVelocity, aEndD, aDeltaS, aDeltaT)));
//}


//----------------------------------------------------------------------------------------------

void TVehicleController::UpdateTrajectory(
    const std::vector<double>& aPreviousPathX,
    const std::vector<double>& aPreviousPathY,
    const CarState& aCarState,
    std::vector<double>& aNextPathX,
    std::vector<double>& aNextPathY,
    const std::vector<std::vector<double>>& aSensorFusionData)
{
  const int n = int(mTimeHorizon * 1000) / 20;
  // std::cout << "NumCars: "  << aSensorFusionData.size() << std::endl;


  const int kPathSize = aPreviousPathX.size();
  for (int i = 0; i < kPathSize; ++i)
  {
    aNextPathX.push_back(aPreviousPathX[i]);
    aNextPathY.push_back(aPreviousPathY[i]);
  }

  const double kDelayT = kPathSize * 0.02;
  TSensorFusion SensorFusion(aSensorFusionData, mWaypoints, kDelayT);

  if (!mIsInitialized)
  {
    mCurrentTime = 0.0;
    const Eigen::Vector2d kInitialFrenet =
      mWaypoints.CalcFrenet(Eigen::Vector2d(aCarState.x, aCarState.y), aCarState.s);

    mCurrentState << kInitialFrenet(0), 0.0, 0.0, kInitialFrenet(1), 0.0, 0.0;
    mIsInitialized = true;
  }

  mpCurrentTrajectory = mStateMachine.Execute(mCurrentState, mCurrentTime, SensorFusion);

  for (int i = kPathSize; i < n; ++i)
  {
    mCurrentState = mpCurrentTrajectory->EvalAt(mCurrentTime);
    const auto p = mWaypoints.getXY_interpolated(mCurrentState(0), mCurrentState(3));
    // std::cout << p(0) << ", " << p(1) << std::endl;
    aNextPathX.push_back(p(0));
    aNextPathY.push_back(p(1));
    mCurrentTime += 0.02;
  }
}


//==============================================================================================
