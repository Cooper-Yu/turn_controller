/** @file
 * @brief Angle-boundary and actuator-limit regression tests. */
#include <gtest/gtest.h>

#include <limits>

#include "turn_controller/turn_math.hpp"
using turn_controller::AngularPid;
using turn_controller::yaw_increment;

TEST(Angle, PositiveBoundary)
{
  EXPECT_NEAR(yaw_increment(-3.13, 3.13), 0.02318530718, 1e-9);
}

TEST(Angle, NegativeBoundary)
{
  EXPECT_NEAR(yaw_increment(3.13, -3.13), -0.02318530718, 1e-9);
}

TEST(Pid, SignedAndSlewLimited)
{
  AngularPid p(2.0, .03, .25, .6, .6);
  EXPECT_NEAR(p.update(-.785, 0, .02), -.012, 1e-9);
  for (int i = 0; i < 100; ++i) EXPECT_LE(std::abs(p.update(-.785, 0, .02)), .600001);
}

TEST(Pid, ResetAndInvalidInterval)
{
  AngularPid p(2.0, .03, .25, .6, .6);
  p.update(1, 0, .1);
  p.reset();
  EXPECT_NEAR(p.update(-1, 0, .02), -.012, 1e-9);
  EXPECT_EQ(p.update(1, 0, 0), 0);
  EXPECT_EQ(p.update(std::numeric_limits<double>::quiet_NaN(), 0, .02), 0);
}

TEST(Pid, DampingOpposesMeasuredMotion)
{
  AngularPid p(1, 0, .5, 1, 100);
  EXPECT_LT(p.update(0, .2, .1), 0);
}

TEST(Pid, NoIntegralWindupWhileSaturated)
{
  AngularPid p(2, 1, 0, .6, 100);
  for (int i = 0; i < 100; ++i) p.update(1, 0, .02);
  EXPECT_NEAR(p.update(0, 0, .02), 0, 1e-9);
}

#include "turn_controller/turn_route.hpp"

TEST(Route, DegreesAndRadiansHaveTheSameMeaning)
{
  const auto degrees = turn_controller::turn_by_degrees(-45.0);
  const auto radians = turn_controller::turn_by_radians(-0.7853981633974483);
  EXPECT_DOUBLE_EQ(degrees.angle_rad, radians.angle_rad);
}

TEST(Route, FreezeAllWaypointsFromOneOrigin)
{
  const double origin = turn_controller::turn_by_degrees(30.0).angle_rad;
  const auto points =
    turn_controller::build_turn_waypoints(1.2, -0.4, origin, turn_controller::default_turn_route());
  ASSERT_EQ(points.size(), 4u);
  const double expected[] = {-15.0, 30.0, 75.0, 30.0};
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(points[i].x_m, 1.2);
    EXPECT_DOUBLE_EQ(points[i].y_m, -0.4);
    EXPECT_NEAR(points[i].yaw_rad, expected[i] * 3.14159265358979323846 / 180.0, 1e-12);
  }
  EXPECT_NEAR(points.back().yaw_rad, origin, 1e-12);
}

TEST(Route, PreserveLongSignedArcAcrossPi)
{
  const auto route = std::vector<turn_controller::TurnStep>{
    turn_controller::turn_by_degrees(270.0), turn_controller::turn_by_degrees(-360.0)};
  const auto points = turn_controller::build_turn_waypoints(0, 0, 3.1, route);
  EXPECT_NEAR(points[0].yaw_rad, 3.1 + 1.5 * 3.14159265358979323846, 1e-12);
  EXPECT_LT(points[1].yaw_rad, points[0].yaw_rad);
}

TEST(Route, RejectInvalidDescriptionsAndOrigins)
{
  EXPECT_THROW(turn_controller::turn_by_degrees(361), std::invalid_argument);
  EXPECT_THROW(turn_controller::turn_by_radians(0), std::invalid_argument);
  EXPECT_THROW(
    turn_controller::turn_by_radians(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(turn_controller::build_turn_waypoints(0, 0, 0, {}), std::invalid_argument);
  EXPECT_THROW(turn_controller::build_turn_waypoints(0, 0, 0, {{7.0}}), std::invalid_argument);
  EXPECT_THROW(
    turn_controller::build_turn_waypoints(
      std::numeric_limits<double>::quiet_NaN(), 0, 0, turn_controller::default_turn_route()),
    std::invalid_argument);
}