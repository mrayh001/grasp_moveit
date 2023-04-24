#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <functional>

using moveit::planning_interface::MoveGroupInterface;

void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg, std::shared_ptr<MoveGroupInterface> move_group_interface, const rclcpp::Logger& logger) {
  // Set the target pose for your robot
  move_group_interface->setPoseTarget(*msg);

  // Create a plan to the target pose using a lambda function
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface->plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    move_group_interface->execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}

int main(int argc, char* argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "grasp_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("grasp_moveit");

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = std::make_shared<MoveGroupInterface>(node, "panda_arm");

  // Create a subscriber to listen for target_pose updates
  auto target_pose_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose", 10, 
    [move_group_interface, logger](const geometry_msgs::msg::Pose::SharedPtr msg) {
      targetPoseCallback(msg, move_group_interface, logger);
    });

  // Use rclcpp::spin to keep the node alive and process callbacks
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}