//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef KALMANFILTER_H
#define KALMANFILTER_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <deque>
//==============================================================================================

class TKalmanFilter
{
public:
  TKalmanFilter();
  TKalmanFilter(double s, double d, double vs);

public:
  Eigen::VectorXd Predict(double dt);
  void Update(double s, double d, double vs);
  double S() const;
  double D() const;
  double Vs() const;
  const Eigen::VectorXd& X() const;


public:
  void PushState();
  void PopState();

private:
  void UpdateQ(double dt);
  void UpdateF(double dt);


private:
  Eigen::VectorXd mX;
  Eigen::MatrixXd mP;

  std::deque<Eigen::VectorXd> mStackX;
  std::deque<Eigen::MatrixXd> mStackP;

  Eigen::MatrixXd mQ;
  Eigen::MatrixXd mF;
  Eigen::MatrixXd mH;
  Eigen::MatrixXd mR;
};


//==============================================================================================
#endif // KALMANFILTER_H
//==============================================================================================
