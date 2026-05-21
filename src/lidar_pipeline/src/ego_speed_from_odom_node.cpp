#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

class EgoSpeedFromOdomNode : public rclcpp::Node
{
public:
  EgoSpeedFromOdomNode()
  : Node("ego_speed_from_odom_node")
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");

    this->declare_parameter<bool>("use_speed_magnitude", false);
    this->declare_parameter<double>("smoothing_alpha", 0.35);
    this->declare_parameter<double>("max_speed_mps", 5.0);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    ego_speed_topic_ = this->get_parameter("ego_speed_topic").as_string();

    use_speed_magnitude_ = this->get_parameter("use_speed_magnitude").as_bool();
    smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();
    max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);

    if (max_speed_mps_ <= 0.0) {
      max_speed_mps_ = 5.0;
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      std::bind(&EgoSpeedFromOdomNode::odomCallback, this, std::placeholders::_1));

    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      ego_speed_topic_,
      10);

    RCLCPP_INFO(
      this->get_logger(),
      "Ego speed from odom node started | input=%s | output=%s",
      odom_topic_.c_str(),
      ego_speed_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;

    double speed = 0.0;

    if (use_speed_magnitude_) {
      speed = std::hypot(vx, vy);
    } else {
      speed = vx;
    }

    speed = std::clamp(speed, -max_speed_mps_, max_speed_mps_);

    if (!has_previous_speed_) {
      filtered_speed_mps_ = speed;
      has_previous_speed_ = true;
    } else {
      filtered_speed_mps_ =
        (1.0 - smoothing_alpha_) * filtered_speed_mps_ +
        smoothing_alpha_ * speed;
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(filtered_speed_mps_);
    speed_pub_->publish(out);
  }

private:
  std::string odom_topic_;
  std::string ego_speed_topic_;

  bool use_speed_magnitude_ = false;
  double smoothing_alpha_ = 0.35;
  double max_speed_mps_ = 5.0;

  bool has_previous_speed_ = false;
  double filtered_speed_mps_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoSpeedFromOdomNode>());
  rclcpp::shutdown();
  return 0;
}
