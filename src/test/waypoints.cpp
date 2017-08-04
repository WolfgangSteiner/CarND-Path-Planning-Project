//==============================================================================================
#include "../catch.hpp"
#include "../Waypoints.h"
#include "../Eigen-3.3/Eigen/Core"
//==============================================================================================

static TWaypoints sWaypoints;

static bool check_transformation(double x0, double y0, double s0)
{
  const Eigen::Vector2d f = sWaypoints.CalcFrenet(Eigen::Vector2d(x0,y0), s0);
  const Eigen::Vector2d p1 = sWaypoints.getXY_interpolated(f(0), f(1));

  return p1(0) == Approx(x0) && p1(1) == Approx(y0);
}

TEST_CASE( "Checking coordinate transformation", "[TWaypoints::CalcFrenet]" )
{
  CHECK(check_transformation(909.48, 1128.67, 124.8336));

  CHECK(check_transformation(1350.68, 1181.17, 575.959));
  CHECK(check_transformation(1461.76, 1172.62, 687.457));
  CHECK(check_transformation(1480.65, 1162.23, 707.52));
  CHECK(check_transformation(1346.14, 1177.69, 571.808));
  CHECK(check_transformation(1351.6, 1185.05, 576.517));
  CHECK(check_transformation(1535.15, 1160.7, 761.475));
  CHECK(check_transformation(1250.06, 1188.63, 474.668));
  CHECK(check_transformation(1472.47, 1167.29, 698.779));
  CHECK(check_transformation(1329.1, 1187.95, 553.897));
  CHECK(check_transformation(1370.01, 1175.48, 595.665));
  CHECK(check_transformation(1299.61, 1190.33, 524.168));

  CHECK(check_transformation(2061.31, 1288.08, 1343.05));
  CHECK(check_transformation(2079.23, 1348.05, 1405.28));
  CHECK(check_transformation(2102.27, 1397.58, 1459.25));
  CHECK(check_transformation(2054.81, 1264.58, 1319.54));
  CHECK(check_transformation(2029.58, 1234.6, 1281.73));
  CHECK(check_transformation(2040.65, 1255.75, 1305.42));
  CHECK(check_transformation(2090.62, 1371.38, 1430.78));
  CHECK(check_transformation(2092.32, 1365.34, 1425.48));
  CHECK(check_transformation(2015.1, 1215.38, 1257.99));
  CHECK(check_transformation(2065.72, 1288.84, 1345.33));
  CHECK(check_transformation(1997.29, 1198.18, 1233.57));

  CHECK(check_transformation(2249.04, 2172.28, 2247.93));
  CHECK(check_transformation(2268.76, 2294.48, 2371.53));
  CHECK(check_transformation(2267.37, 2245.69, 2323.08));
  CHECK(check_transformation(2215.59, 2067.63, 2138.41));
  CHECK(check_transformation(2263.33, 2269.61, 2346.15));
  CHECK(check_transformation(2212.71, 2035.64, 2106.35));
  CHECK(check_transformation(2231.5, 2121.33, 2194.55));
  CHECK(check_transformation(2263.36, 2245.96, 2322.69));
  CHECK(check_transformation(2259.33, 2222.12, 2298.63));
  CHECK(check_transformation(2222.59, 2062.33, 2134.87));
  CHECK(check_transformation(2251.83, 2205.63, 2281.13));

  CHECK(check_transformation(2311.71, 2871.18, 2958));
  CHECK(check_transformation(2197.31, 2987.07, 3121.24));
  CHECK(check_transformation(2177.47, 3001.55, 3143.66));
  CHECK(check_transformation(2312.22, 2861.01, 2948.75));
  CHECK(check_transformation(2218.92, 2975.88, 3097.31));
  CHECK(check_transformation(2245.11, 2955.03, 3064.18));
  CHECK(check_transformation(2283.43, 2910.63, 3005.84));
  CHECK(check_transformation(2299.69, 2892.77, 2982.22));
  CHECK(check_transformation(2232.92, 2975.37, 3086.08));
  CHECK(check_transformation(2312.12, 2879.11, 2964.62));
  CHECK(check_transformation(2270.46, 2928.71, 3027.91));

  CHECK(check_transformation(1448.11, 2942.81, 3874.85));
  CHECK(check_transformation(1576.36, 2950.21, 3745.92));
  CHECK(check_transformation(1485.92, 2951.28, 3837));
  CHECK(check_transformation(1503.62, 2947.48, 3819.32));
  CHECK(check_transformation(1597.92, 2952.9, 3724.26));
  CHECK(check_transformation(1619.44, 2955.08, 3702.68));
  CHECK(check_transformation(1577.36, 2958.23, 3743.74));
  CHECK(check_transformation(1431.87, 2950.64, 3891.11));
  CHECK(check_transformation(1615.48, 2960.86, 3706.02));
  CHECK(check_transformation(1538.89, 2953.1, 3783.14));
  CHECK(check_transformation(1640.28, 2957.47, 3681.67));
}
