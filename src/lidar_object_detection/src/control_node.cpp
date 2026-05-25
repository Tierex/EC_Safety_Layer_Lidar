#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    this->declare_parameter<std::string>("input_topic", "/safety_signal");
    this->declare_parameter<std::string>("output_topic", "/cmd_vel");

    this->declare_parameter<double>("speed_free_mps", 1.3889);
    this->declare_parameter<double>("speed_hazard_mps", 0.50);
    this->declare_parameter<double>("speed_emergency_mps", 0.0);

    this->declare_parameter<double>("max_accel_step_mps", 0.20);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    safety_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      input_topic_,
      10,
      std::bind(&ControlNode::safetyCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_topic_,
      10);

    RCLCPP_INFO(
      this->get_logger(),
      "Control node started | input=%s | output=%s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void safetyCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    const int signal = msg->data;

    const double speed_free =
      this->get_parameter("speed_free_mps").as_double();

    const double speed_hazard =
      this->get_parameter("speed_hazard_mps").as_double();

    const double speed_emergency =
      this->get_parameter("speed_emergency_mps").as_double();

    const double max_step =
      std::max(0.0, this->get_parameter("max_accel_step_mps").as_double());

    double target_speed = speed_free;

    if (signal == 0) {
      target_speed = speed_emergency;
    } else if (signal == 1) {
      target_speed = speed_hazard;
    } else {
      target_speed = speed_free;
    }

    if (max_step > 0.0) {
      const double delta = target_speed - current_command_speed_;

      if (delta > max_step) {
        current_command_speed_ += max_step;
      } else if (delta < -max_step) {
        current_command_speed_ -= max_step;
      } else {
        current_command_speed_ = target_speed;
      }
    } else {
      current_command_speed_ = target_speed;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = current_command_speed_;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    cmd_pub_->publish(cmd);
  }

private:
  std::string input_topic_;
  std::string output_topic_;

  double current_command_speed_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
