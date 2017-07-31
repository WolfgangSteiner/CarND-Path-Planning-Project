#include "../catch.hpp"
#include "../Trajectory.h"
#include "../Eigen-3.3/Eigen/Core"

TEST_CASE( "Factors of the derivative are computed", "[Trajectory::SDerivCoeff]" ) {
  REQUIRE( Trajectory::SDerivCoeff(0,0) == 1.0 );
  REQUIRE( Trajectory::SDerivCoeff(0,1) == 0.0 );
  REQUIRE( Trajectory::SDerivCoeff(1,0) == 1.0 );
  REQUIRE( Trajectory::SDerivCoeff(1,1) == 1.0 );
  REQUIRE( Trajectory::SDerivCoeff(1,2) == 0.0 );
  REQUIRE( Trajectory::SDerivCoeff(2,0) == 1.0 );
  REQUIRE( Trajectory::SDerivCoeff(2,1) == 2.0 );
  REQUIRE( Trajectory::SDerivCoeff(2,2) == 2.0 );
  REQUIRE( Trajectory::SDerivCoeff(2,3) == 0.0 );
  REQUIRE( Trajectory::SDerivCoeff(3,0) == 1.0 );
  REQUIRE( Trajectory::SDerivCoeff(3,1) == 3.0 );
  REQUIRE( Trajectory::SDerivCoeff(3,2) == 6.0 );
  REQUIRE( Trajectory::SDerivCoeff(3,3) == 6.0 );
  REQUIRE( Trajectory::SDerivCoeff(3,4) == 0.0 );
  REQUIRE( Trajectory::SDerivCoeff(5,4) == 5.0 * 4.0 * 3.0 * 2.0 );
}

TEST_CASE( "Evaluating polynomial", "[Trajectory::SEvalAt]" )
{
  Eigen::VectorXd c = Eigen::VectorXd(6);
  c.setConstant(1.0);

  REQUIRE(Trajectory::SEvalAt(c, 1.0, 0) == 6.0);
  REQUIRE(Trajectory::SEvalAt(c, 1.0, 1) == 1.0 + 2.0 + 3.0 + 4.0 + 5.0);
  REQUIRE(Trajectory::SEvalAt(c, 1.0, 2) == 2.0 + 6.0 + 12.0 + 20.0);
  REQUIRE(Trajectory::SEvalAt(c, 1.0, 5) == 5.0 * 4.0 * 3.0 * 2.0);
}

TEST_CASE( "Evaluating state", "[Trajectory::SEvalStateAt]" )
{
  Eigen::VectorXd c = Eigen::VectorXd(6);
  c.setConstant(1.0);

  REQUIRE(Trajectory::SEvalStateAt(c, 1.0) == Eigen::Vector3d(6.0, 15.0, 40.0));
}