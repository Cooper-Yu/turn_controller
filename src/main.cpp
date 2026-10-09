/** @file
 * @brief Task3 process entry point. */
#include <iostream>

#include "turn_controller/turn_controller.hpp"

/** @brief Run Task3 until all turns complete or a runtime guard fails.
 * @param[in] argc Process argument count forwarded to ROS initialization.
 * @param[in] argv Process argument strings read by ROS parameter parsing.
 * @return Zero on completion, one on configuration error, two on runtime failure.
 * @note Simulation-only Task3; does not implement Task4 or connect to hardware.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TurnController>();
    rclcpp::spin(node);
    if (rclcpp::ok()) rclcpp::shutdown();
    return node->exit_code();
  } catch (const std::exception & error) {
    std::cerr << "turn_controller: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
