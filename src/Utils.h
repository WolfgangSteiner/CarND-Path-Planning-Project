//==============================================================================================
#ifndef UTILS_H
#define UTILS_H
//==============================================================================================
#include "Waypoint.h"
//==============================================================================================

namespace NUtils
{
  double deg2rad(double x);
  double rad2deg(double x);
  double distance(double x1, double y1, double x2, double y2);
  double distance(const Waypoint& wp1, const Waypoint& wp2);
  int SLaneNumberForD(double d);
  double SDForLaneNumber(int aLaneNumber);
  double SLaneWidth();
}

//==============================================================================================
#endif // UTILS_H
//==============================================================================================
