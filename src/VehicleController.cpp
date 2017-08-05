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
  // std::cout << "NumCars: "  << aSensorFusionData.size() << std::endl;

  const int kPredictionPathSize = int(mPredictionHorizon * 1000) / 20;
  const int kDisplayPathSize = int(mDisplayHorizon * 1000) / 20;

  const int kPreviousPathSize = aPreviousPathX.size();
  const int kPreviousPredictionPathSize = kPreviousPathSize - (kDisplayPathSize - kPredictionPathSize);
  for (int i = 0; i < kPreviousPredictionPathSize; ++i)
  {
    aNextPathX.push_back(aPreviousPathX[i]);
    aNextPathY.push_back(aPreviousPathY[i]);
  }

  const double kDelayT = kPreviousPredictionPathSize * 0.02;
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
  double iCurrentTime = mCurrentTime;
  Eigen::VectorXd iCurrentState = mCurrentState;

  for (int i = kPreviousPredictionPathSize; i < kDisplayPathSize; ++i)
  {
    if (i == kPredictionPathSize)
    {
      mCurrentTime = iCurrentTime;
      mCurrentState = iCurrentState;
    }

    const auto p = mWaypoints.getXY_interpolated(iCurrentState(0), iCurrentState(3));
    iCurrentTime += 0.02;
    iCurrentState = mpCurrentTrajectory->EvalAt(iCurrentTime);

    aNextPathX.push_back(p(0));
    aNextPathY.push_back(p(1));
  }
}


//==============================================================================================
