#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor_node")
  {
    this->declare_parameter<std::string>("tracked_objects_topic", "/tracked_objects");
    this->declare_parameter<std::string>("safety_signal_topic", "/safety_signal");
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");

    this->declare_parameter<double>("print_rate_hz", 2.0);

    tracked_objects_topic_ =
      this->get_parameter("tracked_objects_topic").as_string();

    safety_signal_topic_ =
      this->get_parameter("safety_signal_topic").as_string();

    ego_speed_topic_ =
      this->get_parameter("ego_speed_topic").as_string();

    print_rate_hz_ =
      this->get_parameter("print_rate_hz").as_double();

    if (print_rate_hz_ <= 0.0) {
      print_rate_hz_ = 2.0;
    }

    tracks_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      tracked_objects_topic_,
      10,
      std::bind(&SafetyMonitorNode::tracksCallback, this, std::placeholders::_1));

    safety_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      safety_signal_topic_,
      10,
      std::bind(&SafetyMonitorNode::safetyCallback, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      ego_speed_topic_,
      10,
      std::bind(&SafetyMonitorNode::speedCallback, this, std::placeholders::_1));

    const double period = 1.0 / print_rate_hz_;

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&SafetyMonitorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Safety monitor node started");
  }

private:
  void tracksCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::size_t stride = 0;

    if (msg->layout.dim.size() >= 2 && msg->layout.dim[1].size > 0) {
      stride = msg->layout.dim[1].size;
    }

    if (stride != 19 && stride != 13) {
      if (!msg->data.empty() && msg->data.size() % 19 == 0) {
        stride = 19;
      } else if (!msg->data.empty() && msg->data.size() % 13 == 0) {
        stride = 13;
      } else {
        last_track_count_ = 0;
        min_distance_ = std::numeric_limits<double>::infinity();
        min_ttc_ = std::numeric_limits<double>::infinity();
        return;
      }
    }

    const std::size_t n = msg->data.size() / stride;
    last_track_count_ = n;

    double min_dist = std::numeric_limits<double>::infinity();
    double min_ttc = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t k = i * stride;

      const double x = msg->data[k + 1];
      const double y = msg->data[k + 2];

      double dist = std::hypot(x, y);
      min_dist = std::min(min_dist, dist);

      if (stride == 19) {
        const double closing_speed = msg->data[k + 7];

        if (closing_speed > 0.10) {
          const double ttc = dist / closing_speed;
          min_ttc = std::min(min_ttc, ttc);
        }
      }
    }

    min_distance_ = min_dist;
    min_ttc_ = min_ttc;
  }

  void safetyCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    last_safety_signal_ = msg->data;
    have_safety_ = true;
  }

  void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    ego_speed_mps_ = msg->data;
    have_speed_ = true;
  }

  void timerCallback()
  {
    const std::string signal_text =
      last_safety_signal_ == 0 ? "Emergency" :
      last_safety_signal_ == 1 ? "Hazard" :
      last_safety_signal_ == 2 ? "Free" :
      "Unknown";

    RCLCPP_INFO(
      this->get_logger(),
      "SafetyMonitor | signal=%d (%s) | ego_speed=%.3f m/s | tracks=%zu | min_dist=%.2f m | min_ttc=%.2f s",
      last_safety_signal_,
      signal_text.c_str(),
      ego_speed_mps_,
      last_track_count_,
      std::isfinite(min_distance_) ? min_distance_ : -1.0,
      std::isfinite(min_ttc_) ? min_ttc_ : -1.0);
  }

private:
  std::string tracked_objects_topic_;
  std::string safety_signal_topic_;
  std::string ego_speed_topic_;

  double print_rate_hz_ = 2.0;

  int last_safety_signal_ = -1;
  bool have_safety_ = false;

  float ego_speed_mps_ = 0.0f;
  bool have_speed_ = false;

  std::size_t last_track_count_ = 0;
  double min_distance_ = std::numeric_limits<double>::infinity();
  double min_ttc_ = std::numeric_limits<double>::infinity();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
