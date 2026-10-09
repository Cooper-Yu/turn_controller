/** @file
 * @brief Pure angle and PID calculations for the in-place turn controller.
 */
#ifndef TURN_CONTROLLER__TURN_MATH_HPP_
#define TURN_CONTROLLER__TURN_MATH_HPP_
#include <algorithm>
#include <cmath>

namespace turn_controller
{
/** @brief Return the signed shortest difference between two sampled headings.
 * @param[in] current Latest odom yaw, radians; read without modification.
 * @param[in] previous Previous accepted odom yaw, radians; read without modification.
 * @return Increment in [-pi, pi], accumulated by TurnController::on_odom().
 * @note Sampling must be frequent enough that rotation between samples is below pi.
 */
inline double yaw_increment(double current, double previous)
{
  return std::atan2(std::sin(current - previous), std::cos(current - previous));
}

/** @brief Angular PID with conditional integration and command slew limiting. */
class AngularPid
{
public:
  /** @brief Configure gains and bounds from TurnController parameters.
   * @param[in] kp Proportional gain, read for angular error feedback.
   * @param[in] ki Integral gain, read for accumulated error feedback.
   * @param[in] kd Derivative gain, read for measured yaw-rate damping.
   * @param[in] speed Maximum absolute output angular velocity, rad/s.
   * @param[in] acceleration Maximum output slew, rad/s squared.
   */
  AngularPid(double kp, double ki, double kd, double speed, double acceleration)
  : kp_{kp}, ki_{ki}, kd_{kd}, speed_{speed}, acceleration_{acceleration}
  {
  }

  /** @brief Clear accumulated error and previous output after stopping or changing targets. */
  void reset()
  {
    integral_ = command_ = 0.0;
  }

  /** @brief Compute angular velocity for one valid control interval.
   * @param[in] error Frozen target minus continuous odom heading, radians, from TurnController::execute_current_turn().
   * @param[in] yaw_rate Measured odom angular.z, rad/s, used as derivative-on-measurement.
   * @param[in] dt Positive ROS-time interval supplied by TurnController::execute_current_turn(), seconds.
   * @return Bounded rad/s command returned to TurnController::execute_current_turn() for publishing; zero for invalid input.
   * @note Writes internal integral/output; integration is blocked when pushing saturation.
   */
  double update(double error, double yaw_rate, double dt)
  {
    if (!std::isfinite(error) || !std::isfinite(yaw_rate) || !std::isfinite(dt) || dt <= 0.0) {
      reset();
      return 0.0;
    }
    const double candidate = std::clamp(integral_ + error * dt, -0.5, 0.5);
    const double requested = kp_ * error + ki_ * candidate - kd_ * yaw_rate;
    if (std::abs(requested) <= speed_ || error * requested < 0.0) integral_ = candidate;
    const double limited =
      std::clamp(kp_ * error + ki_ * integral_ - kd_ * yaw_rate, -speed_, speed_);
    command_ += std::clamp(limited - command_, -acceleration_ * dt, acceleration_ * dt);
    return command_;
  }

private:
  double kp_{};            ///< Proportional gain, rad/s per rad.
  double ki_{};            ///< Integral gain, rad/s per rad-second.
  double kd_{};            ///< Rate damping gain, rad/s per rad/s.
  double speed_{};         ///< Absolute angular command bound, rad/s.
  double acceleration_{};  ///< Angular command slew bound, rad/s squared.
  double integral_{};      ///< Bounded integral of angular error, rad-seconds.
  double command_{};       ///< Previous emitted angular velocity, rad/s.
};
}  // namespace turn_controller
#endif
