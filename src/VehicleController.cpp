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

void TVehicleController::UpdateTrajectory(
    const std::vector<double>& aPreviousPathX,
    const std::vector<double>& aPreviousPathY,
    const CarState& aCarState,
    std::vector<double>& aNextPathX,
    std::vector<double>& aNextPathY,
    const std::vector<std::vector<double>>& aSensorFusionData)
{
  if (!mIsInitialized)
  {
    const auto f = mWaypoints.CalcFrenet(Eigen::Vector2d(aCarState.x, aCarState.y), aCarState.s);
    mCurrentState << f(0), 0.0, 0.0, f(1), 0.0, 0.0;
    mIsInitialized = true;
  }

  const int kPredictionPathSize = int(mPredictionHorizon * 1000) / 20;
  const int kDisplayPathSize = int(mDisplayHorizon * 1000) / 20;
  const int kPreviousPathSize = aPreviousPathX.size();
  const int kPreviousPredictionPathSize = std::max(0, kPreviousPathSize - (kDisplayPathSize - kPredictionPathSize));

  for (int i = 0; i < kPreviousPredictionPathSize; ++i)
  {
    aNextPathX.push_back(aPreviousPathX[i]);
    aNextPathY.push_back(aPreviousPathY[i]);
  }

  const double kDelayT = std::max(0.0, kPreviousPredictionPathSize * 0.02);
  mSensorFusion.Update(aSensorFusionData, mCurrentState(0));
  mSensorFusion.Predict(kDelayT);

  mpCurrentTrajectory = mStateMachine.Execute(mCurrentState, mCurrentTime, mSensorFusion);
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
