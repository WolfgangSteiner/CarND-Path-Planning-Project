//==============================================================================================
#include "Utils.h"
//==============================================================================================
#include <assert.h>
#include <cmath>
#include <vector>
//==============================================================================================

double NUtils::deg2rad(double x)
{
  return x * M_PI / 180;
}


//----------------------------------------------------------------------------------------------

double NUtils::rad2deg(double x)
{
  return x * 180 / M_PI;
}


//----------------------------------------------------------------------------------------------

double NUtils::SDistance(double x1, double x2)
{
  return std::abs(x2 - x1);
}


//----------------------------------------------------------------------------------------------

double NUtils::SDistance(double x1, double y1, double x2, double y2)
{
   return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


//----------------------------------------------------------------------------------------------

double NUtils::SDistance(const Waypoint &wp1, const Waypoint &wp2)
{
  return NUtils::SDistance(wp1.x_, wp1.y_, wp2.x_, wp2.y_);
}


//----------------------------------------------------------------------------------------------

int NUtils::SLaneNumberForD(double d)
{
  if (d > 0.0 || d < -12.0)
  {
    return -1;
  }

  return d < -8.0 ? 0 : d < -4.0 ? 1 : 2;
}


//----------------------------------------------------------------------------------------------

double NUtils::SDForLaneNumber(int aLaneNumber)
{
  // Define the rightmost lane slightly to the left in order to avoid
  // "out of lane" simulator bugs due coordinate warping in curves.
  static std::vector<double> sDVector = {-9.75, -6.0, -2.0};
  assert(aLaneNumber >= 0 && aLaneNumber <= 2);
  return sDVector[aLaneNumber];
}


//----------------------------------------------------------------------------------------------

double NUtils::SLaneWidth()
{
  return 4.0;
}


//----------------------------------------------------------------------------------------------

Eigen::VectorXd NUtils::SMakeVector(double x, double y)
{
  Eigen::VectorXd v(2);
  v << x,y;
  return v;
}


//==============================================================================================
