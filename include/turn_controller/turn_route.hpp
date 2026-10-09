/** @file
 * @brief Reusable signed turn descriptions and fixed waypoint generation. */
#ifndef TURN_CONTROLLER__TURN_ROUTE_HPP_
#define TURN_CONTROLLER__TURN_ROUTE_HPP_
#include <cmath>
#include <stdexcept>
#include <vector>

namespace turn_controller
{
/** @brief One signed relative turn; describes intent without publishing a command. */
struct TurnStep
{
  double angle_rad{};  ///< Positive left/counterclockwise, negative right/clockwise, radians.
};

/** @brief An in-place target frozen in the initialized odometry frame. */
struct TurnWaypoint
{
  double x_m{};      ///< Initial odom x in meters; retained for logs, not translation control.
  double y_m{};      ///< Initial odom y in meters; identical for every in-place waypoint.
  double yaw_rad{};  ///< Cumulative unwrapped heading in radians; preserves the requested arc.
  double relative_angle_rad{};  ///< Signed planned rotation from the previous nominal heading.
};

/** @brief Describe a turn using the course's required angle unit.
 * @param[in] angle_rad Signed relative radians from route composition or configure_turns().
 * Read and validate it; no caller state is changed.
 * @return A TurnStep consumed by build_turn_waypoints(); invalid input throws.
 * @note Each step must be finite, nonzero, and within +/-2*pi. No ROS or motion side effects.
 */
inline TurnStep turn_by_radians(double angle_rad)
{
  constexpr double pi = 3.14159265358979323846;
  if (!std::isfinite(angle_rad) || std::abs(angle_rad) < 1e-6 || std::abs(angle_rad) > 2.0 * pi)
    throw std::invalid_argument("Each turn must be finite, nonzero and within +/-2*pi radians");
  return TurnStep{angle_rad};
}

/** @brief Describe the same turn with a degree convenience input.
 * @param[in] angle_deg Signed degrees from route composition; read and convert once to radians.
 * @return Radians-based TurnStep from turn_by_radians(), for build_turn_waypoints().
 * @note Degree input is limited to this named wrapper; ROS turn_angles remains radians-only.
 */
inline TurnStep turn_by_degrees(double angle_deg)
{
  constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
  return turn_by_radians(angle_deg * radians_per_degree);
}

/** @brief Compose the user's right/left/left/right 45-degree demonstration.
 * @return Four reusable descriptions supplied to configure_turns() as ROS parameter defaults.
 * @note Add, remove or reorder factory calls here; the executor needs no new control branch.
 */
inline std::vector<TurnStep> default_turn_route()
{
  return {
    turn_by_degrees(-45.0),
    turn_by_degrees(45.0),
    turn_by_degrees(45.0),
    turn_by_degrees(-45.0),
  };
}

/** @brief Compose Task4's independently adjustable relative steps.
 * @return Three descriptions consumed by configure_turns(): right 30, right 30, left 60 degrees.
 * @note Initial estimates require cloud hardware calibration. Each step shares the same executor.
 */
inline std::vector<TurnStep> real_turn_route()
{
  return {turn_by_degrees(-30.0), turn_by_degrees(-30.0), turn_by_degrees(60.0)};
}

/** @brief Generate every fixed heading from one initialized pose and a sequence of relative turns.
 * @param[in] x_m Initial odom x captured by handle_initialization(); copied to every waypoint.
 * @param[in] y_m Initial odom y captured by handle_initialization(); copied to every waypoint.
 * @param[in] yaw_rad Initial continuous odom yaw from handle_initialization(); read as the first origin.
 * @param[in] steps Route descriptions from configure_turns(); read each angle without modifying them.
 * @return Ordered targets assigned to TurnController::waypoints_ by handle_initialization().
 * @note Targets accumulate planned rotations, not prior stopping errors. No angle wrapping or motion.
 */
inline std::vector<TurnWaypoint> build_turn_waypoints(
  double x_m, double y_m, double yaw_rad, const std::vector<TurnStep> & steps)
{
  if (!std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(yaw_rad) || steps.empty())
    throw std::invalid_argument("Waypoint origin must be finite and the route must not be empty");
  std::vector<TurnWaypoint> points;
  points.reserve(steps.size());
  double target = yaw_rad;
  for (const auto & step : steps) {
    const double angle = turn_by_radians(step.angle_rad).angle_rad;
    target += angle;
    if (!std::isfinite(target)) throw std::invalid_argument("Accumulated heading must be finite");
    points.push_back(TurnWaypoint{x_m, y_m, target, angle});
  }
  return points;
}
}  // namespace turn_controller
#endif
