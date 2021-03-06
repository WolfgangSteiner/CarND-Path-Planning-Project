//==============================================================================================
#ifndef WAYPOINT_H
#define WAYPOINT_H
//==============================================================================================

//==============================================================================================

class Waypoint
{
public:
  Waypoint()
  : x_(0.0), y_(0.0), s_(0.0), dx_(0.0), dy_(0.0)
  {}

  Waypoint(double x, double y, double s, double dx, double dy)
  : x_(x), y_(y), s_(s), dx_(dx), dy_(dy)
  {}

public:
  double x_, y_, s_, dx_, dy_;
};


//==============================================================================================
#endif // WAYPOINT_H
//==============================================================================================
