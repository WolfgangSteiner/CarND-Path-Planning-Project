//==============================================================================================
#ifndef WAYPOINTS_H
#define WAYPOINTS_H
//==============================================================================================
#include <vector>
#include "Waypoint.h"
#include "Eigen-3.3/Eigen/Core"
#include "spline.h"
//==============================================================================================

class TWaypoints
{
public:
  TWaypoints();

public:
  Eigen::Vector2d CalcFrenet(const Eigen::Vector2d& p, double s_start) const;
  Eigen::Vector2d getXY_interpolated(double s, double d) const;
  Eigen::Vector2d GetNormalAt(double s) const;

  const Waypoint& at(int idx) const;

  double MaxS() const;

public:
  std::vector<Waypoint> waypoints_;

private:
  void fit_splines();
  double Error(const Eigen::Vector2d& p, double s) const;
  double ErrorDeriv(const Eigen::Vector2d& p, double s) const;


private:
  tk::spline x_spline_;
  tk::spline y_spline_;
  double max_s_;
};

//==============================================================================================
#endif // WAYPOINTS_H
//==============================================================================================
