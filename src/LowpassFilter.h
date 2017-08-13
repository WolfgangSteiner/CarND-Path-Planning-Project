//==============================================================================================
// Created by Wolfgang Steiner
//==============================================================================================
#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

class TLowpassFilter
{
public:
  TLowpassFilter(double aFilterConstant) : mFilterConstant{aFilterConstant} {}
  void Init(const Eigen::VectorXd& aX);
  Eigen::VectorXd Update(const Eigen::VectorXd& aX);

private:
  Eigen::VectorXd mX;
  double mFilterConstant{0.0};
};


//==============================================================================================
#endif //LOWPASSFILTER_H
//==============================================================================================
