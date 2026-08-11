#include <chrono>
#include <cmath>
#include <functional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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

      const geometry_msgs::msg::Vector3 &t = transform.transform.translation;

      double error_distance = calculate_distance(t);
      double error_yaw = std::atan2(t.y, t.x);

      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = (error_distance - offset_distance_) * kp_distance_;
      msg.angular.z = error_yaw * kp_yaw_;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(),
                  "Publishing: linear.x=%.2f, angular.z = % .2f ", msg.linear.x,
                  msg.angular.z);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not get transform: %s", ex.what());
    }
  }

  double
  calculate_distance(const geometry_msgs::msg::Vector3 &translation_dist) {
    double dx = translation_dist.x;
    double dy = translation_dist.y;
    double dz = translation_dist.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Rick should be more than 0.5 meters from morty
  double offset_distance_ = 0.5;

  // PID gains
  double kp_yaw_ = 1.5;
  double kp_distance_ = 1.5;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RobotChaseNode> chase_node;
  chase_node = std::make_shared<RobotChaseNode>();

  rclcpp::spin(chase_node);
  return 0;
}
