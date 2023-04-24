#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class TargetPosePublisher : public rclcpp::Node
{
public:
  TargetPosePublisher()
    : Node("target_pose_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&TargetPosePublisher::publishTargetPose, this));
  }

private:
  void publishTargetPose()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    publisher_->publish(target_pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

