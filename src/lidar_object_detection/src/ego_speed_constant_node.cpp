#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class EgoSpeedConstantNode : public rclcpp::Node
{
public:
  EgoSpeedConstantNode()
  : Node("ego_speed_constant_node")
  {
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");
    this->declare_parameter<double>("speed_mps", 1.3889);       // 5 km/h
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    topic_ = this->get_parameter("ego_speed_topic").as_string();
    speed_mps_ = this->get_parameter("speed_mps").as_double();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 50.0;
    }

    pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, 10);

    const double period = 1.0 / publish_rate_hz_;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&EgoSpeedConstantNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Ego speed constant node started | topic=%s | speed=%.3f m/s",
      topic_.c_str(),
      speed_mps_);
  }

private:
  void timerCallback()
  {
    speed_mps_ = this->get_parameter("speed_mps").as_double();

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(speed_mps_);
    pub_->publish(msg);
  }

private:
  std::string topic_;
  double speed_mps_ = 1.3889;
  double publish_rate_hz_ = 50.0;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoSpeedConstantNode>());
  rclcpp::shutdown();
  return 0;
}
