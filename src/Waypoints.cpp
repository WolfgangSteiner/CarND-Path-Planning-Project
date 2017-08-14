//======================================================================================================================
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "Waypoints.h"
#include "Waypoint.h"
#include "Utils.h"
#include "spline.h"
//======================================================================================================================
using Eigen::Vector2d;
//======================================================================================================================
static tk::spline sSplineX;
static tk::spline sSplineY;
static bool sIsInitialized = false;

//======================================================================================================================

TWaypoints::TWaypoints()
{
  max_s_ = 6945.554;

  if (!sIsInitialized)
  {
    std::string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    std::string line;

    while (getline(in_map_, line))
    {
    	std::istringstream iss(line);
    	double x, y, s, d_x, d_y;
    	iss >> x >> y >> s >> d_x >> d_y;
      waypoints_.push_back(Waypoint(x, y, s, d_x, d_y));
    }
    fit_splines();
    sIsInitialized = true;
  }
}


//----------------------------------------------------------------------------------------------

double TWaypoints::MaxS() const
{
  return max_s_;
}


//----------------------------------------------------------------------------------------------

const Waypoint& TWaypoints::at(int idx) const
{
  return waypoints_[idx];
}


//----------------------------------------------------------------------------------------------

double TWaypoints::Error(const Eigen::Vector2d& p, double s) const
{
  return pow(p(0) - sSplineX(s), 2) + pow(p(1) - sSplineY(s), 2);
}


//----------------------------------------------------------------------------------------------

double TWaypoints::ErrorDeriv(const Eigen::Vector2d& p, double s) const
{
  return -2.0 * (p(0) - sSplineX(s)) * sSplineX.deriv(1, s)
         - 2.0 * (p(1) - sSplineY(s)) * sSplineY.deriv(1,s);
}


//----------------------------------------------------------------------------------------------

Eigen::Vector2d TWaypoints::CalcFrenet(const Eigen::Vector2d& p, double aStartS) const
{
  // Perform gradient descent in order to find the point on the spline that is closest to p:
  const double eps = 1.0e-6;
  double s = aStartS;
  const double kGamma = 0.001;
  const double kPrecision = 1e-12;
  double PreviousStepSize = s;

  while (PreviousStepSize > kPrecision)
  {
    const double next_s = s - kGamma * ErrorDeriv(p, s);
    PreviousStepSize = std::abs(next_s - s);
    s = next_s;
  }

  // From the established point on the spline find the offset vector to the target
  // point and do a component-wise divide by the normal vector:
  const Vector2d p_spline(sSplineX(s), sSplineY(s));
  const Vector2d p_delta = (p - p_spline).array() / GetNormalAt(s).array();

  // Use the mean of the two resulting components as the d-coordinate:
  const double d = 0.5 * (p_delta(0) + p_delta(1));
  return Vector2d(s, d);
}


//----------------------------------------------------------------------------------------------

Vector2d TWaypoints::GetNormalAt(double s) const
{
  return Vector2d(-sSplineY.deriv(1, s), sSplineX.deriv(1, s));
}


//----------------------------------------------------------------------------------------------

Eigen::Vector2d TWaypoints::getXY_interpolated(double s, double d) const
{
  while (s > max_s_)
  {
    s -= max_s_;
  }

  while (s < 0)
  {
    s += max_s_;
  }

  return Vector2d(sSplineX(s), sSplineY(s)) + GetNormalAt(s) * d;
}


//----------------------------------------------------------------------------------------------

void TWaypoints::fit_splines()
{
  std::vector<double> s_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;

  const Waypoint& first_wp = waypoints_.front();
  const Waypoint& last_wp = waypoints_.back();

  s_vec.push_back(last_wp.s_ - max_s_);
  x_vec.push_back(last_wp.x_);
  y_vec.push_back(last_wp.y_);

  for (const Waypoint& iwp: waypoints_)
  {
    s_vec.push_back(iwp.s_);
    x_vec.push_back(iwp.x_);
    y_vec.push_back(iwp.y_);
  }

  s_vec.push_back(first_wp.s_ + max_s_);
  x_vec.push_back(first_wp.x_);
  y_vec.push_back(first_wp.y_);

  std::cout << "Fitting splines...";
  sSplineX.set_points(s_vec, x_vec);
  sSplineY.set_points(s_vec, y_vec);
  std::cout << " done!" << std::endl;
}


//======================================================================================================================
