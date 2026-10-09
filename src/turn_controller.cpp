/** @file
 * @brief TurnController initialization, waypoint dispatch and angular tracking. */
#include "turn_controller/turn_controller.hpp"

#include <stdexcept>

TurnController::TurnController() : Node{"turn_controller"}
{
  if (get_node_parameters_interface()->get_parameter_overrides().count("use_sim_time") == 0)
    set_parameter(rclcpp::Parameter("use_sim_time", true));
  configure_turns();
  const double kp = positive("kp", 1.8);
  const double ki = nonnegative("ki", 0.03);
  const double kd = nonnegative("kd", 0.35);
  const double speed = positive("max_angular_speed", 0.6);
  const double acceleration = positive("max_angular_acceleration", 0.6);
  pid_ = std::make_unique<turn_controller::AngularPid>(kp, ki, kd, speed, acceleration);
  tolerance_ = positive("heading_tolerance", 0.01);
  stopped_rate_ = positive("stopped_yaw_rate", 0.02);
  settle_time_ = positive("settle_duration", 0.4);
  dwell_time_ = nonnegative("dwell_duration", 0.5);
  segment_timeout_ = positive("segment_timeout", 30.0);
  startup_timeout_ = positive("startup_timeout", 15.0);
  odom_timeout_ = positive("odom_timeout", 0.5);
  pub_ = create_publisher<geometry_msgs::msg::Twist>(
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel"), 10);
  sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter<std::string>("odom_topic", "/odometry/filtered"), rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
  RCLCPP_INFO(
    get_logger(), "Task3: %zu relative turns; PID=(%.3f, %.3f, %.3f), max_wz=%.3f", steps_.size(),
    kp, ki, kd, speed);
}

int TurnController::exit_code() const
{
  return exit_code_;
}

double TurnController::nonnegative(const std::string & name, double fallback)
{
  const double value = declare_parameter<double>(name, fallback);
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(name + " must be finite and nonnegative");
  return value;
}

double TurnController::positive(const std::string & name, double fallback)
{
  const double value = nonnegative(name, fallback);
  if (value == 0.0) throw std::invalid_argument(name + " must be positive");
  return value;
}

void TurnController::on_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto & position = msg.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) return;
  const auto & q = msg.pose.pose.orientation;
  const auto & v = msg.twist.twist;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 || !std::isfinite(v.angular.z) ||
    !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y))
    return;
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (
    stamp.nanoseconds() <= 0 || age < -0.1 || age > odom_timeout_ || (received_ && stamp <= stamp_))
    return;
  const auto receipt = std::chrono::steady_clock::now();
  if (received_ && std::chrono::duration<double>(receipt - receipt_).count() > odom_timeout_) {
    fail("ODOM_TIMEOUT");
    return;
  }
  if (msg.header.frame_id.empty() || msg.child_frame_id.empty()) return;
  double roll{}, pitch{}, yaw{};
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
  if (received_ && (msg.header.frame_id != frame_ || msg.child_frame_id != child_frame_)) {
    fail("ODOM_FRAME_CHANGED");
    return;
  }
  if (received_) {
    const double increment = turn_controller::yaw_increment(yaw, previous_yaw_);
    if (std::abs(increment) > 0.35) {
      fail("ODOM_YAW_JUMP");
      return;
    }
    continuous_yaw_ += increment;
  } else {
    continuous_yaw_ = yaw;
    frame_ = msg.header.frame_id;
    child_frame_ = msg.child_frame_id;
  }
  x_ = position.x;
  y_ = position.y;
  previous_yaw_ = yaw;
  yaw_rate_ = v.angular.z;
  planar_speed_ = std::hypot(v.linear.x, v.linear.y);
  stamp_ = stamp;
  receipt_ = receipt;
  if (std::abs(yaw_rate_) >= stopped_rate_ || planar_speed_ >= 0.01) {
    startup_stable_ = settling_ = dwelling_ = false;
  }
  received_ = true;
}

void TurnController::publish(double wz)
{
  geometry_msgs::msg::Twist command;
  command.angular.z = wz;
  pub_->publish(command);
}

void TurnController::stop()
{
  publish(0.0);
  pid_->reset();
}

void TurnController::fail(const char * reason)
{
  stop();
  exit_code_ = 2;
  RCLCPP_ERROR(get_logger(), "%s: stopped; exiting with status 2", reason);
  rclcpp::shutdown();
}

void TurnController::begin_turn(const rclcpp::Time & time)
{
  target_ = waypoints_.at(index_).yaw_rad;
  segment_started_ = std::chrono::steady_clock::now();
  last_tick_ = time;
  settling_ = dwelling_ = false;
  pid_->reset();
  RCLCPP_INFO(
    get_logger(), "Turn %zu/%zu: start=%.6f delta=%.6f target=%.6f rad (unwrapped)", index_ + 1,
    steps_.size(), continuous_yaw_, steps_[index_].angle_rad, target_);
}

bool TurnController::handle_completion(const rclcpp::Time & time, double error)
{
  if (std::abs(error) > tolerance_) {
    settling_ = dwelling_ = false;
    return false;
  }
  stop();
  if (std::abs(yaw_rate_) >= stopped_rate_ || planar_speed_ >= 0.01) {
    settling_ = dwelling_ = false;
    return true;
  }
  if (!settling_) {
    settling_ = true;
    hold_since_ = time;
  }
  if (!dwelling_ && (time - hold_since_).seconds() >= settle_time_) {
    dwelling_ = true;
    hold_since_ = time;
    RCLCPP_INFO(
      get_logger(), "Turn %zu settled: error=%.6f wz=%.6f; dwell=%.2f s", index_ + 1, error,
      yaw_rate_, dwell_time_);
  }
  if (!dwelling_ || (time - hold_since_).seconds() < dwell_time_) return true;
  RCLCPP_INFO(
    get_logger(), "Reached turn %zu: target=%.6f actual=%.6f error=%.6f rad", index_ + 1, target_,
    continuous_yaw_, error);
  ++index_;
  if (index_ == steps_.size()) {
    RCLCPP_INFO(get_logger(), "All turns completed; stopped.");
    rclcpp::shutdown();
  } else
    begin_turn(time);
  return true;
}

void TurnController::configure_turns()
{
  const auto defaults = turn_controller::default_turn_route();
  std::vector<double> angles;
  for (const auto & step : defaults) angles.push_back(step.angle_rad);
  angles = declare_parameter<std::vector<double>>("turn_angles", angles);
  if (angles.empty()) throw std::invalid_argument("turn_angles must not be empty");
  for (const double angle : angles) steps_.push_back(turn_controller::turn_by_radians(angle));
}

bool TurnController::handle_feedback_guard(const std::chrono::steady_clock::time_point & wall)
{
  if (!received_) {
    stop();
    if (std::chrono::duration<double>(wall - startup_started_).count() > startup_timeout_)
      fail("INITIAL_ODOM_TIMEOUT");
    return true;
  }
  if (std::chrono::duration<double>(wall - receipt_).count() > odom_timeout_) {
    fail("ODOM_TIMEOUT");
    return true;
  }
  return false;
}

bool TurnController::handle_initialization(
  const std::chrono::steady_clock::time_point & wall, const rclcpp::Time & time)
{
  if (started_) return false;
  stop();
  if (std::chrono::duration<double>(wall - startup_started_).count() > startup_timeout_) {
    fail("INITIAL_STOP_TIMEOUT");
    return true;
  }
  if (std::abs(yaw_rate_) >= stopped_rate_ || planar_speed_ >= 0.01) {
    startup_stable_ = false;
    return true;
  }
  if (!startup_stable_) {
    startup_stable_ = true;
    startup_stable_stamp_ = stamp_;
  }
  if ((stamp_ - startup_stable_stamp_).seconds() < 0.3) return true;
  waypoints_ = turn_controller::build_turn_waypoints(x_, y_, continuous_yaw_, steps_);
  started_ = true;
  RCLCPP_INFO(
    get_logger(), "Initialization complete: x=%.6f y=%.6f yaw=%.6f rad, odom_frame=%s", x_, y_,
    continuous_yaw_, frame_.c_str());
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & point = waypoints_[i];
    RCLCPP_INFO(
      get_logger(), "Waypoint %zu: x=%.6f y=%.6f yaw=%.6f rad (unwrapped), delta=%.6f rad", i + 1,
      point.x_m, point.y_m, point.yaw_rad, point.relative_angle_rad);
  }
  begin_turn(time);
  return true;
}

void TurnController::execute_current_turn(
  const std::chrono::steady_clock::time_point & wall, const rclcpp::Time & time)
{
  if (std::chrono::duration<double>(wall - segment_started_).count() > segment_timeout_) {
    fail("TURN_TIMEOUT");
    return;
  }
  const double dt = (time - last_tick_).seconds();
  if (dt == 0.0) return;
  if (dt < 0.0 || dt > 0.5) {
    fail("ROS_TIME_JUMP");
    return;
  }
  last_tick_ = time;
  const double error = target_ - continuous_yaw_;
  if (!handle_completion(time, error)) {
    const double command = pid_->update(error, yaw_rate_, dt);
    publish(command);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Turn progress %zu/%zu: yaw=%.4f target=%.4f error=%.4f rad measured_wz=%.4f "
      "command_wz=%.4f rad/s dt=%.3f s",
      index_ + 1, steps_.size(), continuous_yaw_, target_, error, yaw_rate_, command, dt);
  }
}

void TurnController::tick()
{
  const auto wall = std::chrono::steady_clock::now();
  const auto time = now();
  if (handle_feedback_guard(wall) || handle_initialization(wall, time)) return;
  execute_current_turn(wall, time);
}
