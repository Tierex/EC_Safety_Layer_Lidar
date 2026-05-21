#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

class EgoYawFromImuNode : public rclcpp::Node
{
public:
  EgoYawFromImuNode()
  : Node("ego_yaw_from_imu_node")
  {
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("ego_yaw_rate_topic", "/ego_yaw_rate");

    this->declare_parameter<double>("smoothing_alpha", 0.35);
    this->declare_parameter<double>("max_yaw_rate_rps", 10.0);

    imu_topic_ = this->get_parameter("imu_topic").as_string();
    ego_yaw_rate_topic_ = this->get_parameter("ego_yaw_rate_topic").as_string();

    smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();
    max_yaw_rate_rps_ = this->get_parameter("max_yaw_rate_rps").as_double();

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);

    if (max_yaw_rate_rps_ <= 0.0) {
      max_yaw_rate_rps_ = 10.0;
    }

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      10,
      std::bind(&EgoYawFromImuNode::imuCallback, this, std::placeholders::_1));

    yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      ego_yaw_rate_topic_,
      10);

    RCLCPP_INFO(
      this->get_logger(),
      "Ego yaw from IMU node started | input=%s | output=%s",
      imu_topic_.c_str(),
      ego_yaw_rate_topic_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double yaw_rate = msg->angular_velocity.z;

    yaw_rate = std::clamp(
      yaw_rate,
      -max_yaw_rate_rps_,
      max_yaw_rate_rps_);

    if (!has_previous_yaw_) {
      filtered_yaw_rate_rps_ = yaw_rate;
      has_previous_yaw_ = true;
    } else {
      filtered_yaw_rate_rps_ =
        (1.0 - smoothing_alpha_) * filtered_yaw_rate_rps_ +
        smoothing_alpha_ * yaw_rate;
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(filtered_yaw_rate_rps_);
    yaw_pub_->publish(out);
  }

private:
  std::string imu_topic_;
  std::string ego_yaw_rate_topic_;

  double smoothing_alpha_ = 0.35;
  double max_yaw_rate_rps_ = 10.0;

  bool has_previous_yaw_ = false;
  double filtered_yaw_rate_rps_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoYawFromImuNode>());
  rclcpp::shutdown();
  return 0;
}
