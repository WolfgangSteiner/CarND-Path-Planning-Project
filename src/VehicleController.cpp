//==============================================================================================
#include "VehicleController.h"
#include "CarState.h"
#include "KeepLaneState.h"
#include "SensorFusion.h"
#include "Utils.h"
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
, mLowpassFilter{0.9}
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
    // mLowpassFilter.Init(NUtils::SMakeVector(aCarState.x, aCarState.y));
    mCurrentState << f(0), 0.0, 0.0, f(1), 0.0, 0.0;
    mIsInitialized = true;
  }

  const int kPredictionPathSize = int(mPredictionHorizon * 1000) / 20;
  const int kDisplayPathSize = int(mDisplayHorizon * 1000) / 20;
  const int kPreviousPathSize = aPreviousPathX.size();
  const int kPreviousPredictionPathSize = std::max(0, kPreviousPathSize - (kDisplayPathSize - kPredictionPathSize));

  for (int i = 0; i < kPreviousPredictionPathSize; ++i)
  {
    const double x = aPreviousPathX[i];
    const double y = aPreviousPathY[i];
    aNextPathX.push_back(x);
    aNextPathY.push_back(y);
   // mLowpassFilter.Init(NUtils::SMakeVector(x,y));
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

    Eigen::VectorXd p = mWaypoints.getXY_interpolated(iCurrentState(0), iCurrentState(3));
    //p = mLowpassFilter.Update(p);
    iCurrentTime += 0.02;
    iCurrentState = mpCurrentTrajectory->EvalAt(iCurrentTime);

    aNextPathX.push_back(p(0));
    aNextPathY.push_back(p(1));
  }
}


//==============================================================================================
