//==============================================================================================
#ifndef UTILS_H
#define UTILS_H
//==============================================================================================
#include "Waypoint.h"
//==============================================================================================
#include "Eigen-3.3/Eigen/Core"
//==============================================================================================

namespace NUtils
{
  double deg2rad(double x);
  double rad2deg(double x);
  double SDistance(double x1, double y1, double x2, double y2);
  double SDistance(const Waypoint &wp1, const Waypoint &wp2);
  double SDistance(double x1, double x2);
  int SLaneNumberForD(double d);
  double SDForLaneNumber(int aLaneNumber);
  double SLaneWidth();
  Eigen::VectorXd SMakeVector(double x, double y);
}

//==============================================================================================
#endif // UTILS_H
//==============================================================================================
