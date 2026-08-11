#include <chrono>
#include <functional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RobotChaseNode : public rclcpp::Node {
public:
  RobotChaseNode()
      : Node("robot_chase_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);
    auto timer_period = std::chrono::milliseconds(500);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&RobotChaseNode::timer_callback, this));
  }

private:
  void timer_callback() {
    try {
      // Target frame: rick/base_link
      // Source frame: morty/base_link
      auto transform = tf_buffer_.lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);

      const auto &t = transform.transform.translation;
      const auto &q = transform.transform.rotation;

      RCLCPP_INFO(this->get_logger(),
                  "Translation: [%.3f, %.3f, %.3f], "
                  "Rotation: [%.3f, %.3f, %.3f, %.3f]",
                  t.x, t.y, t.z, q.x, q.y, q.z, q.w);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not get transform: %s", ex.what());
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RobotChaseNode> chase_node;
  chase_node = std::make_shared<RobotChaseNode>();

  rclcpp::spin(chase_node);
  return 0;
}
