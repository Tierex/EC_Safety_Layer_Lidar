// ============================================================================
// safety_performance_monitor.cpp
// ============================================================================
// Monitoring node for comparing the custom ROS2 safety pipeline with another
// software route such as Autoware.
//
// Measures:
// - Topic period / frequency
// - Message age where timestamps are available
// - Track count
// - Detector/tracker processing time if available in tracked_objects stride 19
// - Safety signal counts and transitions
// - Emergency reaction time: safety_signal == 0 -> final speed close to zero
// - Command override behaviour: raw Xbox command vs final command
// - Basic system CPU load
// - Monitor process memory usage
// - Optional CSV logging
//
// Intended comparison:
// - Run this node with the custom pipeline
// - Run the same node with Autoware-compatible topic remappings
// - Replay the same rosbag in both runs
// ============================================================================
 
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
 
#include <unistd.h>
 
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
 
using std::placeholders::_1;
 
class RunningStats
{
public:
  void add(double x)
  {
    if (!std::isfinite(x)) {
      return;
    }
 
    count_++;
 
    if (x < min_) {
      min_ = x;
    }
 
    if (x > max_) {
      max_ = x;
    }
 
    const double delta = x - mean_;
    mean_ += delta / static_cast<double>(count_);
 
    const double delta2 = x - mean_;
    m2_ += delta * delta2;
  }
 
  void reset()
  {
    count_ = 0;
    mean_ = 0.0;
    m2_ = 0.0;
    min_ = std::numeric_limits<double>::infinity();
    max_ = -std::numeric_limits<double>::infinity();
  }
 
  std::uint64_t count() const
  {
    return count_;
  }
 
  double mean() const
  {
    if (count_ == 0) {
      return nan();
    }
 
    return mean_;
  }
 
  double min() const
  {
    if (count_ == 0) {
      return nan();
    }
 
    return min_;
  }
 
  double max() const
  {
    if (count_ == 0) {
      return nan();
    }
 
    return max_;
  }
 
  double stddev() const
  {
    if (count_ < 2) {
      return nan();
    }
 
    return std::sqrt(m2_ / static_cast<double>(count_ - 1));
  }
 
private:
  static double nan()
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
 
  std::uint64_t count_ = 0;
  double mean_ = 0.0;
  double m2_ = 0.0;
  double min_ = std::numeric_limits<double>::infinity();
  double max_ = -std::numeric_limits<double>::infinity();
};
 
class SafetyPerformanceMonitor : public rclcpp::Node
{
public:
  SafetyPerformanceMonitor()
  : Node("safety_performance_monitor")
  {
    declareParams();
    loadParams();
    setupCsv();
 
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SafetyPerformanceMonitor::pointcloudCallback, this, _1));
 
    tracked_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      tracked_objects_topic_,
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
      std::bind(&SafetyPerformanceMonitor::trackedObjectsCallback, this, _1));
 
    safety_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      safety_signal_topic_,
      10,
      std::bind(&SafetyPerformanceMonitor::safetySignalCallback, this, _1));
 
    raw_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_topic_,
      10,
      std::bind(&SafetyPerformanceMonitor::rawCmdCallback, this, _1));
 
    final_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      final_cmd_topic_,
      10,
      std::bind(&SafetyPerformanceMonitor::finalCmdCallback, this, _1));
 
    summary_pub_ = this->create_publisher<std_msgs::msg::String>(
      summary_topic_,
      10);
 
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(summary_period_sec_)),
      std::bind(&SafetyPerformanceMonitor::timerCallback, this));
 
    RCLCPP_INFO(this->get_logger(), "SafetyPerformanceMonitor started");
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic:      %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "tracked_objects_topic: %s", tracked_objects_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "safety_signal_topic:   %s", safety_signal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "raw_cmd_topic:         %s", raw_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "final_cmd_topic:       %s", final_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "summary_topic:         %s", summary_topic_.c_str());
  }
 
private:
  // ==========================================================================
  // Parameters
  // ==========================================================================
  void declareParams()
  {
    this->declare_parameter<std::string>("pointcloud_topic", "/velodyne_points");
    this->declare_parameter<std::string>("tracked_objects_topic", "/tracked_objects");
    this->declare_parameter<std::string>("safety_signal_topic", "/safety_signal");
 
    // Pas deze eventueel aan naar jullie echte controller topics.
    this->declare_parameter<std::string>("raw_cmd_topic", "/cmd_vel_raw");
    this->declare_parameter<std::string>("final_cmd_topic", "/cmd_vel");
 
    this->declare_parameter<std::string>("summary_topic", "/safety_monitor_summary");
 
    this->declare_parameter<double>("summary_period_sec", 1.0);
 
    this->declare_parameter<double>("stop_speed_threshold_mps", 0.03);
    this->declare_parameter<double>("override_speed_epsilon_mps", 0.05);
 
    // Leeg laten betekent: geen CSV schrijven.
    this->declare_parameter<std::string>("csv_path", "");
  }
 
  void loadParams()
  {
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    tracked_objects_topic_ = this->get_parameter("tracked_objects_topic").as_string();
    safety_signal_topic_ = this->get_parameter("safety_signal_topic").as_string();
 
    raw_cmd_topic_ = this->get_parameter("raw_cmd_topic").as_string();
    final_cmd_topic_ = this->get_parameter("final_cmd_topic").as_string();
 
    summary_topic_ = this->get_parameter("summary_topic").as_string();
 
    summary_period_sec_ = this->get_parameter("summary_period_sec").as_double();
    stop_speed_threshold_mps_ = this->get_parameter("stop_speed_threshold_mps").as_double();
    override_speed_epsilon_mps_ =
      this->get_parameter("override_speed_epsilon_mps").as_double();
 
    csv_path_ = this->get_parameter("csv_path").as_string();
 
    if (summary_period_sec_ <= 0.0) {
      summary_period_sec_ = 1.0;
    }
 
    stop_speed_threshold_mps_ = std::max(0.0, stop_speed_threshold_mps_);
    override_speed_epsilon_mps_ = std::max(0.0, override_speed_epsilon_mps_);
  }
 
  // ==========================================================================
  // Time helpers
  // ==========================================================================
  double nowSec() const
  {
    return this->now().seconds();
  }
 
  double durationMs(double newer, double older) const
  {
    return (newer - older) * 1000.0;
  }
 
  static double safeHzFromPeriodMs(double period_ms)
  {
    if (!std::isfinite(period_ms) || period_ms <= 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    return 1000.0 / period_ms;
  }
 
  // ==========================================================================
  // CSV
  // ==========================================================================
  void setupCsv()
  {
    if (csv_path_.empty()) {
      return;
    }
 
    bool exists = false;
    {
      std::ifstream test(csv_path_);
      exists = test.good();
    }
 
    csv_.open(csv_path_, std::ios::app);
 
    if (!csv_.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not open csv_path: %s",
        csv_path_.c_str());
      return;
    }
 
    if (!exists) {
      csv_
        << "time_sec,"
        << "pc_hz,pc_age_mean_ms,"
        << "tracked_hz,tracked_age_mean_ms,track_count,"
        << "detector_ms_mean,tracker_ms_mean,"
        << "safety_hz,current_signal,count_emergency,count_hazard,count_free,signal_transitions,"
        << "raw_cmd_hz,final_cmd_hz,raw_speed,final_speed,override_events,"
        << "reaction_mean_ms,reaction_min_ms,reaction_max_ms,"
        << "system_cpu_percent,monitor_rss_mb"
        << "\n";
    }
  }
 
  // ==========================================================================
  // Topic callbacks
  // ==========================================================================
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double now = nowSec();
 
    if (last_pointcloud_time_sec_ > 0.0) {
      pointcloud_period_ms_.add(durationMs(now, last_pointcloud_time_sec_));
    }
 
    last_pointcloud_time_sec_ = now;
    pointcloud_count_total_++;
 
    const rclcpp::Time stamp(msg->header.stamp);
 
    if (stamp.nanoseconds() > 0) {
      const double age_ms = (this->now() - stamp).seconds() * 1000.0;
      pointcloud_age_ms_.add(age_ms);
    }
  }
 
  void trackedObjectsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const double now = nowSec();
 
    if (last_tracked_time_sec_ > 0.0) {
      tracked_period_ms_.add(durationMs(now, last_tracked_time_sec_));
    }
 
    last_tracked_time_sec_ = now;
    tracked_count_total_++;
 
    std::size_t stride = 0;
 
    if (msg->layout.dim.size() >= 2 && msg->layout.dim[1].size > 0) {
      stride = msg->layout.dim[1].size;
    }
 
    if (stride != 13 && stride != 19) {
      if (!msg->data.empty() && msg->data.size() % 19 == 0) {
        stride = 19;
      } else if (!msg->data.empty() && msg->data.size() % 13 == 0) {
        stride = 13;
      }
    }
 
    if (stride != 13 && stride != 19) {
      current_track_count_ = 0;
      return;
    }
 
    const std::size_t n = msg->data.size() / stride;
    current_track_count_ = static_cast<int>(n);
 
    if (stride == 19 && n > 0) {
      const std::size_t k = 0;
 
      const double source_time_sec = static_cast<double>(msg->data[k + 16]);
      const double tracker_processing_ms = static_cast<double>(msg->data[k + 17]);
      const double detector_processing_ms = static_cast<double>(msg->data[k + 18]);
 
      if (source_time_sec > 0.0) {
        tracked_age_ms_.add((now - source_time_sec) * 1000.0);
      }
 
      if (std::isfinite(tracker_processing_ms) && tracker_processing_ms >= 0.0) {
        tracker_processing_ms_.add(tracker_processing_ms);
      }
 
      if (std::isfinite(detector_processing_ms) && detector_processing_ms >= 0.0) {
        detector_processing_ms_.add(detector_processing_ms);
      }
    }
  }
 
  void safetySignalCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    const double now = nowSec();
 
    if (last_safety_time_sec_ > 0.0) {
      safety_period_ms_.add(durationMs(now, last_safety_time_sec_));
    }
 
    last_safety_time_sec_ = now;
    safety_count_total_++;
 
    const int signal = static_cast<int>(msg->data);
 
    if (signal == 0) {
      count_emergency_++;
    } else if (signal == 1) {
      count_hazard_++;
    } else if (signal == 2) {
      count_free_++;
    }
 
    if (have_current_signal_ && signal != current_signal_) {
      signal_transitions_++;
    }
 
    current_signal_ = signal;
    have_current_signal_ = true;
 
    if (signal == 0) {
      emergency_request_time_sec_ = now;
      waiting_for_stop_after_emergency_ = true;
    }
  }
 
  void rawCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double now = nowSec();
 
    if (last_raw_cmd_time_sec_ > 0.0) {
      raw_cmd_period_ms_.add(durationMs(now, last_raw_cmd_time_sec_));
    }
 
    last_raw_cmd_time_sec_ = now;
    raw_cmd_count_total_++;
 
    raw_speed_mps_ = msg->linear.x;
    have_raw_cmd_ = true;
  }
 
  void finalCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double now = nowSec();
 
    if (last_final_cmd_time_sec_ > 0.0) {
      final_cmd_period_ms_.add(durationMs(now, last_final_cmd_time_sec_));
    }
 
    last_final_cmd_time_sec_ = now;
    final_cmd_count_total_++;
 
    final_speed_mps_ = msg->linear.x;
    have_final_cmd_ = true;
 
    if (waiting_for_stop_after_emergency_ &&
        std::abs(final_speed_mps_) <= stop_speed_threshold_mps_)
    {
      const double reaction_ms =
        durationMs(now, emergency_request_time_sec_);
 
      reaction_time_ms_.add(reaction_ms);
      waiting_for_stop_after_emergency_ = false;
    }
 
    if (have_raw_cmd_ && have_current_signal_) {
      const bool raw_wants_motion =
        std::abs(raw_speed_mps_) > stop_speed_threshold_mps_;
 
      const bool final_reduced =
        std::abs(final_speed_mps_) + override_speed_epsilon_mps_ <
        std::abs(raw_speed_mps_);
 
      if ((current_signal_ == 0 || current_signal_ == 1) &&
          raw_wants_motion &&
          final_reduced)
      {
        override_events_++;
      }
    }
  }
 
  // ==========================================================================
  // CPU and memory
  // ==========================================================================
  bool readProcStat(std::uint64_t & idle, std::uint64_t & total) const
  {
    std::ifstream file("/proc/stat");
 
    if (!file.is_open()) {
      return false;
    }
 
    std::string cpu;
    std::uint64_t user = 0;
    std::uint64_t nice = 0;
    std::uint64_t system = 0;
    std::uint64_t idle_time = 0;
    std::uint64_t iowait = 0;
    std::uint64_t irq = 0;
    std::uint64_t softirq = 0;
    std::uint64_t steal = 0;
 
    file >> cpu >> user >> nice >> system >> idle_time >> iowait >> irq >> softirq >> steal;
 
    if (cpu != "cpu") {
      return false;
    }
 
    idle = idle_time + iowait;
    total = user + nice + system + idle_time + iowait + irq + softirq + steal;
 
    return true;
  }
 
  double updateSystemCpuPercent()
  {
    std::uint64_t idle = 0;
    std::uint64_t total = 0;
 
    if (!readProcStat(idle, total)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    if (!have_cpu_sample_) {
      last_cpu_idle_ = idle;
      last_cpu_total_ = total;
      have_cpu_sample_ = true;
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    const std::uint64_t idle_delta = idle - last_cpu_idle_;
    const std::uint64_t total_delta = total - last_cpu_total_;
 
    last_cpu_idle_ = idle;
    last_cpu_total_ = total;
 
    if (total_delta == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    const double busy_delta =
      static_cast<double>(total_delta - idle_delta);
 
    return 100.0 * busy_delta / static_cast<double>(total_delta);
  }
 
  double getOwnRssMb() const
  {
    std::ifstream file("/proc/self/statm");
 
    if (!file.is_open()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    long total_pages = 0;
    long resident_pages = 0;
 
    file >> total_pages >> resident_pages;
 
    const long page_size = sysconf(_SC_PAGESIZE);
 
    if (page_size <= 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
 
    const double bytes =
      static_cast<double>(resident_pages) *
      static_cast<double>(page_size);
 
    return bytes / (1024.0 * 1024.0);
  }
 
  // ==========================================================================
  // Summary
  // ==========================================================================
  std::string fmt(double v, int precision = 3) const
  {
    if (!std::isfinite(v)) {
      return "nan";
    }
 
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << v;
    return ss.str();
  }
 
  void timerCallback()
  {
    const double system_cpu_percent = updateSystemCpuPercent();
    const double own_rss_mb = getOwnRssMb();
 
    const double pc_hz = safeHzFromPeriodMs(pointcloud_period_ms_.mean());
    const double tracked_hz = safeHzFromPeriodMs(tracked_period_ms_.mean());
    const double safety_hz = safeHzFromPeriodMs(safety_period_ms_.mean());
    const double raw_cmd_hz = safeHzFromPeriodMs(raw_cmd_period_ms_.mean());
    const double final_cmd_hz = safeHzFromPeriodMs(final_cmd_period_ms_.mean());
 
    std::ostringstream ss;
 
    ss
      << "Safety monitor summary\n"
      << "----------------------\n"
      << "pointcloud_hz: " << fmt(pc_hz) << "\n"
      << "pointcloud_age_mean_ms: " << fmt(pointcloud_age_ms_.mean()) << "\n"
      << "tracked_hz: " << fmt(tracked_hz) << "\n"
      << "tracked_age_mean_ms: " << fmt(tracked_age_ms_.mean()) << "\n"
      << "track_count: " << current_track_count_ << "\n"
      << "detector_processing_mean_ms: " << fmt(detector_processing_ms_.mean()) << "\n"
      << "tracker_processing_mean_ms: " << fmt(tracker_processing_ms_.mean()) << "\n"
      << "safety_hz: " << fmt(safety_hz) << "\n"
      << "current_signal: " << current_signal_ << "\n"
      << "count_emergency: " << count_emergency_ << "\n"
      << "count_hazard: " << count_hazard_ << "\n"
      << "count_free: " << count_free_ << "\n"
      << "signal_transitions: " << signal_transitions_ << "\n"
      << "raw_cmd_hz: " << fmt(raw_cmd_hz) << "\n"
      << "final_cmd_hz: " << fmt(final_cmd_hz) << "\n"
      << "raw_speed_mps: " << fmt(raw_speed_mps_) << "\n"
      << "final_speed_mps: " << fmt(final_speed_mps_) << "\n"
      << "override_events: " << override_events_ << "\n"
      << "reaction_mean_ms: " << fmt(reaction_time_ms_.mean()) << "\n"
      << "reaction_min_ms: " << fmt(reaction_time_ms_.min()) << "\n"
      << "reaction_max_ms: " << fmt(reaction_time_ms_.max()) << "\n"
      << "system_cpu_percent: " << fmt(system_cpu_percent) << "\n"
      << "monitor_rss_mb: " << fmt(own_rss_mb) << "\n";
 
    std_msgs::msg::String out;
    out.data = ss.str();
    summary_pub_->publish(out);
 
    if (debug_) {
      RCLCPP_INFO(
        this->get_logger(),
        "pc_hz=%s tracked_hz=%s safety_hz=%s signal=%d tracks=%d reaction_mean_ms=%s cpu=%s rss=%s",
        fmt(pc_hz).c_str(),
        fmt(tracked_hz).c_str(),
        fmt(safety_hz).c_str(),
        current_signal_,
        current_track_count_,
        fmt(reaction_time_ms_.mean()).c_str(),
        fmt(system_cpu_percent).c_str(),
        fmt(own_rss_mb).c_str());
    }
 
    writeCsvRow(system_cpu_percent, own_rss_mb);
  }
 
  void writeCsvRow(double system_cpu_percent, double own_rss_mb)
  {
    if (!csv_.is_open()) {
      return;
    }
 
    csv_
      << fmt(nowSec(), 6) << ","
      << fmt(safeHzFromPeriodMs(pointcloud_period_ms_.mean())) << ","
      << fmt(pointcloud_age_ms_.mean()) << ","
      << fmt(safeHzFromPeriodMs(tracked_period_ms_.mean())) << ","
      << fmt(tracked_age_ms_.mean()) << ","
      << current_track_count_ << ","
      << fmt(detector_processing_ms_.mean()) << ","
      << fmt(tracker_processing_ms_.mean()) << ","
      << fmt(safeHzFromPeriodMs(safety_period_ms_.mean())) << ","
      << current_signal_ << ","
      << count_emergency_ << ","
      << count_hazard_ << ","
      << count_free_ << ","
      << signal_transitions_ << ","
      << fmt(safeHzFromPeriodMs(raw_cmd_period_ms_.mean())) << ","
      << fmt(safeHzFromPeriodMs(final_cmd_period_ms_.mean())) << ","
      << fmt(raw_speed_mps_) << ","
      << fmt(final_speed_mps_) << ","
      << override_events_ << ","
      << fmt(reaction_time_ms_.mean()) << ","
      << fmt(reaction_time_ms_.min()) << ","
      << fmt(reaction_time_ms_.max()) << ","
      << fmt(system_cpu_percent) << ","
      << fmt(own_rss_mb)
      << "\n";
 
    csv_.flush();
  }
 
  // ==========================================================================
  // Parameters
  // ==========================================================================
  std::string pointcloud_topic_;
  std::string tracked_objects_topic_;
  std::string safety_signal_topic_;
  std::string raw_cmd_topic_;
  std::string final_cmd_topic_;
  std::string summary_topic_;
  std::string csv_path_;
 
  double summary_period_sec_ = 1.0;
  double stop_speed_threshold_mps_ = 0.03;
  double override_speed_epsilon_mps_ = 0.05;
 
  // ==========================================================================
  // ROS
  // ==========================================================================
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tracked_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr final_cmd_sub_;
 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
 
  // ==========================================================================
  // Timing stats
  // ==========================================================================
  double last_pointcloud_time_sec_ = -1.0;
  double last_tracked_time_sec_ = -1.0;
  double last_safety_time_sec_ = -1.0;
  double last_raw_cmd_time_sec_ = -1.0;
  double last_final_cmd_time_sec_ = -1.0;
 
  RunningStats pointcloud_period_ms_;
  RunningStats pointcloud_age_ms_;
 
  RunningStats tracked_period_ms_;
  RunningStats tracked_age_ms_;
  RunningStats detector_processing_ms_;
  RunningStats tracker_processing_ms_;
 
  RunningStats safety_period_ms_;
  RunningStats raw_cmd_period_ms_;
  RunningStats final_cmd_period_ms_;
 
  RunningStats reaction_time_ms_;
 
  // ==========================================================================
  // Counters and state
  // ==========================================================================
  std::uint64_t pointcloud_count_total_ = 0;
  std::uint64_t tracked_count_total_ = 0;
  std::uint64_t safety_count_total_ = 0;
  std::uint64_t raw_cmd_count_total_ = 0;
  std::uint64_t final_cmd_count_total_ = 0;
 
  std::uint64_t count_emergency_ = 0;
  std::uint64_t count_hazard_ = 0;
  std::uint64_t count_free_ = 0;
  std::uint64_t signal_transitions_ = 0;
  std::uint64_t override_events_ = 0;
 
  int current_signal_ = 2;
  bool have_current_signal_ = false;
 
  int current_track_count_ = 0;
 
  double raw_speed_mps_ = 0.0;
  double final_speed_mps_ = 0.0;
 
  bool have_raw_cmd_ = false;
  bool have_final_cmd_ = false;
 
  bool waiting_for_stop_after_emergency_ = false;
  double emergency_request_time_sec_ = -1.0;
 
  // ==========================================================================
  // CPU
  // ==========================================================================
  bool have_cpu_sample_ = false;
  std::uint64_t last_cpu_idle_ = 0;
  std::uint64_t last_cpu_total_ = 0;
 
  // ==========================================================================
  // CSV
  // ==========================================================================
  std::ofstream csv_;
 
  // ==========================================================================
  // Debug
  // ==========================================================================
  bool debug_ = true;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyPerformanceMonitor>());
  rclcpp::shutdown();
  return 0;
}