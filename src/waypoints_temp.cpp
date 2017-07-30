//======================================================================================================================
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "Waypoints.h"
#include "Waypoint.h"
#include "utils.h"
//======================================================================================================================
using Eigen::Vector2d;
//======================================================================================================================

Waypoints::Waypoints()
{
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  max_s_ = 6945.554;

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
}


//----------------------------------------------------------------------------------------------

const Waypoint& Waypoints::at(int idx) const
{
  return waypoints_[idx];
}


//----------------------------------------------------------------------------------------------

int Waypoints::ClosestWaypoint(double x, double y) const
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < waypoints_.size(); i++)
	{
		const double map_x = at(i).x_;
		const double map_y = at(i).y_;
		const double dist = NUtils::distance(x, y, map_x, map_y);

		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}


//----------------------------------------------------------------------------------------------

int Waypoints::NextWaypoint(double x, double y, double theta) const
{
	int closest_idx = ClosestWaypoint(x,y);

  const Waypoint& wp = at(closest_idx);
	const double heading = atan2(wp.y_ - y, wp.x_ - x);
	const double angle = std::abs(theta - heading);

	if(angle > M_PI/4)
	{
		closest_idx++;
	}

	return closest_idx;
}


//----------------------------------------------------------------------------------------------

int Waypoints::PreviousWaypoint(int i) const
{
  return i == 0 ? waypoints_.size() - 1 : i - 1;
}


//----------------------------------------------------------------------------------------------

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Vector2d Waypoints::CalcFrenet(const Vector2d& p, double theta) const
{
	const int idx_next_wp = NextWaypoint(p(0),p(1), theta);
	const int idx_prev_wp = PreviousWaypoint(idx_next_wp);

  const Waypoint& next_wp = at(idx_next_wp);
  const Waypoint& prev_wp = at(idx_prev_wp);

	const double n_x = next_wp.x_ - prev_wp.x_;
	const double n_y = next_wp.y_ - prev_wp.y_;
	const double x_x = p(0) - prev_wp.x_;
	const double x_y = p(1) - prev_wp.y_;

	// find the projection of x onto n
	const double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
	const double proj_x = proj_norm * n_x;
	const double proj_y = proj_norm * n_y;

	//see if d value is positive or negative by comparing it to a center point
	const double center_x = 1000-prev_wp.x_;
	const double center_y = 2000-prev_wp.y_;
	const double centerToPos = NUtils::distance(center_x,center_y,x_x,x_y);
	const double centerToRef = NUtils::distance(center_x,center_y,proj_x,proj_y);

	double frenet_d = NUtils::distance(x_x, x_y, proj_x, proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < idx_prev_wp; i++)
	{
		frenet_s += NUtils::distance(at(i), at(i+1));
	}

	frenet_s += NUtils::distance(0,0,proj_x,proj_y);

  return Vector2d(frenet_s, frenet_d);
}


//----------------------------------------------------------------------------------------------
// Transform from Frenet s,d coordinates to Cartesian x,y
Vector2d Waypoints::getXY(double s, double d) const
{
	int idx_prev = -1;

	while(s > at(idx_prev+1).s_ && (idx_prev < (int)(waypoints_.size()-1) ))
	{
		idx_prev++;
	}

  idx_prev = (idx_prev + waypoints_.size()) % waypoints_.size();
  const int idx_next = (idx_prev + 1) % waypoints_.size();

  const Waypoint& prev_wp = at(idx_prev);
  const Waypoint& next_wp = at(idx_next);

	const double heading = atan2(next_wp.y_ - prev_wp.y_, next_wp.x_ - prev_wp.x_);

	// the x,y,s along the segment
	const double seg_s = (s - prev_wp.s_);
	const double seg_x = prev_wp.x_ + seg_s * cos(heading);
	const double seg_y = prev_wp.y_ + seg_s * sin(heading);

	const double perp_heading = heading - M_PI/2;
	const double x = seg_x + d * cos(perp_heading);
	const double y = seg_y + d * sin(perp_heading);

	return Vector2d(x,y);
}


//----------------------------------------------------------------------------------------------

Eigen::Vector2d Waypoints::getXY_interpolated(double s, double d) const
{
  while (s > max_s_)
  {
    s -= max_s_;
  }

  while (s < 0)
  {
    s += max_s_;
  }

  const Vector2d n(-y_spline_.deriv(1, s), x_spline_.deriv(1, s));
  const Vector2d p(x_spline_(s), y_spline_(s));

  return p + n * d;
}


//----------------------------------------------------------------------------------------------

void Waypoints::fit_splines()
{
  std::vector<double> s_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;

  for (const Waypoint& iwp: waypoints_)
  {
    s_vec.push_back(iwp.s_);
    x_vec.push_back(iwp.x_);
    y_vec.push_back(iwp.y_);
  }


  std::cout << "Fitting splines...";
  x_spline_.set_points(s_vec, x_vec);
  y_spline_.set_points(s_vec, y_vec);
  std::cout << " done!" << std::endl;
}


//======================================================================================================================
