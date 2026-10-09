/** @file
 * @brief Select the scene and optional one-time preparation before starting angular control. */
#include <unistd.h>

#include <filesystem>
#include <iostream>

#include "turn_controller/turn_controller.hpp"

/** @brief Replace this process with the preparation coordinator before ROS interfaces exist.
 * @param[in] argc Argument count from main(); bounds copying of ROS overrides.
 * @param[in] argv Arguments from main(); ROS arguments are forwarded to both child controllers.
 * @throws std::runtime_error When process replacement fails.
 * @note Scene 2 calls this once; the coordinator launches the core with --skip-preparation.
 */
void prepare_real_start(int argc, char ** argv)
{
  const auto path =
    std::filesystem::read_symlink("/proc/self/exe").parent_path() / "prepare_and_turn";
  std::vector<std::string> args{path.string(), "--scene", "2"};
  bool ros_args = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--ros-args") ros_args = true;
    if (ros_args) args.emplace_back(argv[i]);
  }
  std::vector<char *> pointers;
  for (auto & arg : args) pointers.push_back(arg.data());
  pointers.push_back(nullptr);
  execv(path.c_str(), pointers.data());
  throw std::runtime_error("Cannot start preparation coordinator");
}

/** @brief Run a scene with optional initial placement, followed by reusable angular-only control.
 * @param[in] argc Process argument count from the shell, read for scene/options and ROS initialization.
 * @param[in] argv Shell arguments: scene 1/2, optional --skip-preparation, and ROS overrides.
 * @return Zero on completion, one on invalid configuration, two on runtime failure.
 * @note Skipping placement still requires fresh stopped odometry at the current point.
 */
int main(int argc, char ** argv)
{
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    int scene = 1;
    bool skip = false;
    bool selected = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--skip-preparation")
        skip = true;
      else if (!selected && (args[i] == "1" || args[i] == "2")) {
        scene = args[i] == "2" ? 2 : 1;
        selected = true;
      } else
        throw std::invalid_argument("Expected scene 1/2 and optional --skip-preparation");
    }
    if (scene == 2 && !skip) prepare_real_start(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurnController>(scene);
    rclcpp::spin(node);
    if (rclcpp::ok()) rclcpp::shutdown();
    return node->exit_code();
  } catch (const std::exception & error) {
    std::cerr << "turn_controller: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
