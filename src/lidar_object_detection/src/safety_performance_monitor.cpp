// ============================================================================
// safety_performance_monitor.cpp
// ============================================================================
// Monitoring node for comparing the custom ROS2 safety pipeline with another
// software route such as Autoware.
//
// Behaviour:
// - Waits until an active "ros2 bag play ..." process is detected.
// - Starts measuring only after a rosbag is detected.
// - Automatically creates output files named after the rosbag.
// - Stops automatically when the rosbag process ends.
// - Writes CSV output.
// - Optional human-readable TXT output can be enabled with enable_txt_output.
// - Does not use shell grep; it scans /proc directly, so no grep process is
//   falsely detected.
//
// Measures:
// - Pointcloud topic frequency and message age
// - Tracked object topic frequency and track count
// - Detector/tracker processing time if available in stride 19
// - Safety signal frequency, counts, transitions
// - Raw and final command frequency
// - Raw and final speed
// - Override events
// - Emergency-to-stop reaction time
// - System CPU usage
// - Monitor process RSS memory
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <dirent.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

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
    RCLCPP_INFO(this->get_logger(), "Waiting for active ros2 bag play process...");
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic:      %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "tracked_objects_topic: %s", tracked_objects_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "safety_signal_topic:   %s", safety_signal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "raw_cmd_topic:         %s", raw_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "final_cmd_topic:       %s", final_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "summary_topic:         %s", summary_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_dir:            %s", output_dir_.c_str());
  }

  ~SafetyPerformanceMonitor() override
  {
    finalizeOutputFiles();
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

    this->declare_parameter<std::string>("raw_cmd_topic", "/cmd_vel_raw");
    this->declare_parameter<std::string>("final_cmd_topic", "/cmd_vel");

    this->declare_parameter<std::string>("summary_topic", "/safety_monitor_summary");

    this->declare_parameter<double>("summary_period_sec", 1.0);

    this->declare_parameter<double>("stop_speed_threshold_mps", 0.03);
    this->declare_parameter<double>("override_speed_epsilon_mps", 0.05);

    this->declare_parameter<std::string>("output_dir", "/home/school/performance_results");

    // Number of monitor cycles after rosbag disappears before shutdown.
    this->declare_parameter<int>("bag_missing_cycles_before_stop", 3);

    this->declare_parameter<bool>("debug", true);

    // CSV is always written. TXT is optional.
    this->declare_parameter<bool>("enable_txt_output", false);

    // Live testing: false.
    // Rosbag replay with --clock: true.
    this->declare_parameter<bool>("enable_sim_time", false);

    // Only accept tracked age when source_time_sec appears to be in the same
    // time domain as nowSec(). Set to 0.0 to disable this upper limit.
    this->declare_parameter<double>("max_tracked_age_ms", 5000.0);
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

    stop_speed_threshold_mps_ =
      this->get_parameter("stop_speed_threshold_mps").as_double();

    override_speed_epsilon_mps_ =
      this->get_parameter("override_speed_epsilon_mps").as_double();

    output_dir_ = this->get_parameter("output_dir").as_string();

    bag_missing_cycles_before_stop_ =
      this->get_parameter("bag_missing_cycles_before_stop").as_int();

    debug_ = this->get_parameter("debug").as_bool();
    enable_txt_output_ = this->get_parameter("enable_txt_output").as_bool();
    enable_sim_time_ = this->get_parameter("enable_sim_time").as_bool();

    max_tracked_age_ms_ = this->get_parameter("max_tracked_age_ms").as_double();

    applySimTimeSetting();

    if (summary_period_sec_ <= 0.0) {
      summary_period_sec_ = 1.0;
    }

    stop_speed_threshold_mps_ = std::max(0.0, stop_speed_threshold_mps_);
    override_speed_epsilon_mps_ = std::max(0.0, override_speed_epsilon_mps_);

    if (output_dir_.empty()) {
      output_dir_ = "/home/school/performance_results";
    }

    if (bag_missing_cycles_before_stop_ < 1) {
      bag_missing_cycles_before_stop_ = 1;
    }

    if (max_tracked_age_ms_ < 0.0) {
      max_tracked_age_ms_ = 0.0;
    }
  }

  void applySimTimeSetting()
  {
    // ROS 2 uses the standard "use_sim_time" parameter to make this->now()
    // follow /clock. This monitor exposes "enable_sim_time" because the
    // workflow switches between live tests and rosbag replay.
    try {
      if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter<bool>("use_sim_time", enable_sim_time_);
      }

      this->set_parameter(
        rclcpp::Parameter(
          "use_sim_time",
          enable_sim_time_));

      RCLCPP_INFO(
        this->get_logger(),
        "Sim time mode: %s",
        enable_sim_time_ ? "enabled" : "disabled");
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      try {
        this->set_parameter(
          rclcpp::Parameter(
            "use_sim_time",
            enable_sim_time_));
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not set use_sim_time after declaration: %s",
          e.what());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not apply sim time setting: %s",
        e.what());
    }
  }

  // ==========================================================================
  // General helpers
  // ==========================================================================
  double nowSec() const
  {
    return this->now().seconds();
  }

  double elapsedMonitorTimeSec() const
  {
    if (!monitor_start_wall_time_set_) {
      return 0.0;
    }

    const auto now = std::chrono::steady_clock::now();

    const double elapsed =
      std::chrono::duration<double>(
        now - monitor_start_wall_time_).count();

    if (!std::isfinite(elapsed) || elapsed < 0.0) {
      return 0.0;
    }

    return elapsed;
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

  bool isReasonableTrackedAgeMs(double age_ms) const
  {
    if (!std::isfinite(age_ms) || age_ms < 0.0) {
      return false;
    }

    if (max_tracked_age_ms_ <= 0.0) {
      return true;
    }

    return age_ms <= max_tracked_age_ms_;
  }

  std::string fmt(double v, int precision = 3) const
  {
    if (!std::isfinite(v)) {
      return "nan";
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << v;
    return ss.str();
  }

  std::string formatLocalTime(
    const std::chrono::system_clock::time_point & time_point,
    const char * format) const
  {
    const std::time_t time = std::chrono::system_clock::to_time_t(time_point);

    std::tm tm_struct;
    localtime_r(&time, &tm_struct);

    char buffer[64] = {0};

    if (std::strftime(buffer, sizeof(buffer), format, &tm_struct) == 0) {
      return std::string();
    }

    return std::string(buffer);
  }

  bool ensureDirectoryExists(const std::string & directory) const
  {
    if (directory.empty()) {
      return false;
    }

    struct stat st;

    if (stat(directory.c_str(), &st) == 0) {
      return S_ISDIR(st.st_mode);
    }

    if (mkdir(directory.c_str(), 0775) != 0) {
      if (errno != EEXIST) {
        return false;
      }
    }

    return true;
  }

  std::string sanitizeFileName(const std::string & raw_name) const
  {
    std::string name = raw_name;

    const auto slash = name.find_last_of("/\\");

    if (slash != std::string::npos) {
      name = name.substr(slash + 1);
    }

    const auto dot = name.find_last_of('.');

    if (dot != std::string::npos && dot != 0) {
      name = name.substr(0, dot);
    }

    std::string clean;

    for (char c : name) {
      const unsigned char uc = static_cast<unsigned char>(c);

      if (std::isalnum(uc) || c == '_' || c == '-') {
        clean.push_back(c);
      } else if (!clean.empty() && clean.back() != '_') {
        clean.push_back('_');
      }
    }

    while (!clean.empty() && clean.back() == '_') {
      clean.pop_back();
    }

    return clean.empty() ? "rosbag" : clean;
  }

  std::string baseNameFromPath(const std::string & raw_path) const
  {
    if (raw_path.empty()) {
      return std::string();
    }

    std::string path = raw_path;

    while (path.size() > 1 && (path.back() == '/' || path.back() == '\\')) {
      path.pop_back();
    }

    const auto sep = path.find_last_of("/\\");

    if (sep == std::string::npos) {
      return path;
    }

    return path.substr(sep + 1);
  }

  // ==========================================================================
  // Rosbag process detection through /proc
  // ==========================================================================
  struct RosbagProcessInfo
  {
    bool running = false;
    std::string bag_path;
    std::string bag_name;
  };

  std::vector<std::string> readProcessArguments(const std::string & pid) const
  {
    std::vector<std::string> args;

    std::string cmdline_path = "/proc/";
    cmdline_path += pid;
    cmdline_path += "/cmdline";

    std::ifstream cmdline_file(cmdline_path, std::ios::binary);

    if (!cmdline_file.is_open()) {
      return args;
    }

    std::string arg;

    while (std::getline(cmdline_file, arg, '\0')) {
      if (!arg.empty()) {
        args.push_back(arg);
      }
    }

    return args;
  }

  std::string readProcessCwd(const std::string & pid) const
  {
    std::string cwd_link = "/proc/";
    cwd_link += pid;
    cwd_link += "/cwd";

    char buffer[4096] = {0};
    const ssize_t len = readlink(cwd_link.c_str(), buffer, sizeof(buffer) - 1);

    if (len <= 0) {
      return std::string();
    }

    buffer[len] = '\0';
    return std::string(buffer);
  }

  bool isAbsolutePath(const std::string & path) const
  {
    return !path.empty() && path[0] == '/';
  }

  bool pathExists(const std::string & path) const
  {
    if (path.empty()) {
      return false;
    }

    struct stat st;
    return stat(path.c_str(), &st) == 0;
  }

  std::string makeAbsolutePath(
    const std::string & path,
    const std::string & cwd) const
  {
    if (path.empty() || isAbsolutePath(path) || cwd.empty()) {
      return path;
    }

    std::string result = cwd;

    if (!result.empty() && result.back() != '/') {
      result += '/';
    }

    result += path;
    return result;
  }

  bool isNumber(const std::string & text) const
  {
    if (text.empty()) {
      return false;
    }

    char * end_ptr = nullptr;
    std::strtod(text.c_str(), &end_ptr);

    return end_ptr != text.c_str() && *end_ptr == '\0';
  }

  bool optionTakesValue(const std::string & option) const
  {
    std::string opt = option;

    const auto eq = opt.find('=');

    if (eq != std::string::npos) {
      opt = opt.substr(0, eq);
    }

    return
      opt == "-r" ||
      opt == "--rate" ||
      opt == "-s" ||
      opt == "--storage" ||
      opt == "--storage-id" ||
      opt == "--read-ahead-queue-size" ||
      opt == "--storage-config-file" ||
      opt == "--start-offset" ||
      opt == "--duration" ||
      opt == "--delay" ||
      opt == "--clock-publish-frequency" ||
      opt == "--playback-until-timestamp" ||
      opt == "--qos-profile-overrides-path" ||
      opt == "--message-order" ||
      opt == "--log-level";
  }

  int findPlayIndex(const std::vector<std::string> & args) const
  {
    for (std::size_t i = 0; i + 1 < args.size(); ++i) {
      if (args[i] == "bag" && args[i + 1] == "play") {
        return static_cast<int>(i + 1);
      }
    }

    for (std::size_t i = 0; i < args.size(); ++i) {
      if (args[i] == "play") {
        for (const auto & arg : args) {
          if (arg.find("rosbag2_transport") != std::string::npos) {
            return static_cast<int>(i);
          }
        }
      }
    }

    return -1;
  }

  std::string extractBagPathAfterPlay(
    const std::vector<std::string> & args,
    int play_index,
    const std::string & cwd) const
  {
    std::vector<std::string> candidates;

    for (std::size_t i = static_cast<std::size_t>(play_index + 1);
         i < args.size();
         ++i)
    {
      const std::string & token = args[i];

      if (token.empty()) {
        continue;
      }

      if (token == "--ros-args") {
        break;
      }

      if (token.rfind("--", 0) == 0 && token.find('=') != std::string::npos) {
        continue;
      }

      if (token == "--clock") {
        if (i + 1 < args.size() && isNumber(args[i + 1])) {
          ++i;
        }
        continue;
      }

      if (token[0] == '-') {
        if (optionTakesValue(token) && i + 1 < args.size()) {
          ++i;
        }
        continue;
      }

      candidates.push_back(token);

      const std::string absolute_candidate = makeAbsolutePath(token, cwd);

      if (pathExists(absolute_candidate)) {
        return token;
      }
    }

    if (!candidates.empty()) {
      return candidates.front();
    }

    return std::string();
  }

  RosbagProcessInfo findActiveRosbagProcess()
  {
    RosbagProcessInfo info;

    DIR * proc_dir = opendir("/proc");

    if (!proc_dir) {
      return info;
    }

    struct dirent * entry = nullptr;

    while ((entry = readdir(proc_dir)) != nullptr) {
      if (entry->d_type != DT_DIR) {
        continue;
      }

      const char * dname = entry->d_name;

      if (!std::isdigit(static_cast<unsigned char>(dname[0]))) {
        continue;
      }

      const std::vector<std::string> args = readProcessArguments(dname);

      if (args.empty()) {
        continue;
      }

      const int play_index = findPlayIndex(args);

      if (play_index < 0) {
        continue;
      }

      const std::string cwd = readProcessCwd(dname);
      const std::string bag_path = extractBagPathAfterPlay(args, play_index, cwd);

      if (bag_path.empty()) {
        continue;
      }

      info.running = true;
      info.bag_path = bag_path;
      info.bag_name = baseNameFromPath(bag_path);

      closedir(proc_dir);
      return info;
    }

    closedir(proc_dir);
    return info;
  }

  // ==========================================================================
  // Output file handling
  // ==========================================================================
  std::string buildOutputPath(
    const std::string & extension,
    const std::string & stop_time) const
  {
    std::ostringstream filename;

    filename
      << rosbag_base_name_
      << "_"
      << start_date_str_
      << "_"
      << start_time_str_;

    if (stop_time.empty()) {
      filename << "_running";
    } else {
      filename << "_" << stop_time;
    }

    filename << extension;

    std::string path = output_dir_;

    if (!path.empty() && path.back() != '/') {
      path += '/';
    }

    path += filename.str();

    return path;
  }

  void startMonitoringForBag(const RosbagProcessInfo & bag)
  {
    rosbag_base_name_ = sanitizeFileName(
      bag.bag_name.empty() ? bag.bag_path : bag.bag_name);

    const auto now = std::chrono::system_clock::now();
    start_date_str_ = formatLocalTime(now, "%Y%m%d");
    start_time_str_ = formatLocalTime(now, "%H%M%S");

    resetAllStats();

    monitor_start_wall_time_ = std::chrono::steady_clock::now();
    monitor_start_wall_time_set_ = true;

    if (!ensureDirectoryExists(output_dir_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not create output directory: %s",
        output_dir_.c_str());
      return;
    }

    csv_running_path_ = buildOutputPath(".csv", "");
    csv_.open(csv_running_path_, std::ios::out);

    if (enable_txt_output_) {
      txt_running_path_ = buildOutputPath(".txt", "");
      txt_.open(txt_running_path_, std::ios::out);
    }

    if (!csv_.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open CSV output file: %s",
        csv_running_path_.c_str());
      return;
    }

    if (enable_txt_output_ && !txt_.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open TXT output file: %s",
        txt_running_path_.c_str());
      return;
    }

    writeCsvHeader();

    if (enable_txt_output_ && txt_.is_open()) {
      txt_ << "Safety performance monitor\n";
      txt_ << "==========================\n";
      txt_ << "rosbag_name: " << rosbag_base_name_ << "\n";
      txt_ << "rosbag_path: " << bag.bag_path << "\n";
      txt_ << "start_date: " << start_date_str_ << "\n";
      txt_ << "start_time: " << start_time_str_ << "\n";
      txt_ << "\n";
      txt_.flush();
    }

    monitoring_active_ = true;
    bag_currently_running_ = true;
    missing_bag_cycles_ = 0;

    RCLCPP_INFO(
      this->get_logger(),
      "Started monitoring rosbag [%s]",
      rosbag_base_name_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "CSV output: %s",
      csv_running_path_.c_str());

    if (enable_txt_output_ && txt_.is_open()) {
      RCLCPP_INFO(
        this->get_logger(),
        "TXT output: %s",
        txt_running_path_.c_str());
    }
  }

  void finalizeOutputFiles()
  {
    if (!monitoring_active_ && csv_running_path_.empty() && txt_running_path_.empty()) {
      return;
    }

    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }

    if (txt_.is_open()) {
      txt_.flush();
      txt_.close();
    }

    const std::string stop_time =
      formatLocalTime(std::chrono::system_clock::now(), "%H%M%S");

    if (!csv_running_path_.empty()) {
      const std::string csv_final_path = buildOutputPath(".csv", stop_time);

      if (std::rename(csv_running_path_.c_str(), csv_final_path.c_str()) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved CSV: %s", csv_final_path.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not rename CSV file from %s",
          csv_running_path_.c_str());
      }
    }

    if (enable_txt_output_ && !txt_running_path_.empty()) {
      const std::string txt_final_path = buildOutputPath(".txt", stop_time);

      if (std::rename(txt_running_path_.c_str(), txt_final_path.c_str()) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved TXT: %s", txt_final_path.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not rename TXT file from %s",
          txt_running_path_.c_str());
      }
    }

    csv_running_path_.clear();
    txt_running_path_.clear();
    monitoring_active_ = false;
  }

  void writeCsvHeader()
  {
    csv_
      << "time_sec,"
      << "bag_name,"
      << "pc_hz,pc_age_mean_ms,"
      << "tracked_hz,tracked_age_mean_ms,track_count,"
      << "detector_ms_mean,tracker_ms_mean,"
      << "safety_hz,current_signal,count_emergency,count_hazard,count_free,signal_transitions,"
      << "raw_cmd_hz,final_cmd_hz,raw_speed,final_speed,override_events,"
      << "reaction_mean_ms,reaction_min_ms,reaction_max_ms,"
      << "system_cpu_percent,monitor_rss_mb"
      << "\n";
  }

  // ==========================================================================
  // Reset
  // ==========================================================================
  void resetAllStats()
  {
    pointcloud_period_ms_.reset();
    pointcloud_age_ms_.reset();

    tracked_period_ms_.reset();
    tracked_age_ms_.reset();
    detector_processing_ms_.reset();
    tracker_processing_ms_.reset();

    safety_period_ms_.reset();
    raw_cmd_period_ms_.reset();
    final_cmd_period_ms_.reset();

    reaction_time_ms_.reset();

    last_pointcloud_time_sec_ = -1.0;
    last_tracked_time_sec_ = -1.0;
    last_safety_time_sec_ = -1.0;
    last_raw_cmd_time_sec_ = -1.0;
    last_final_cmd_time_sec_ = -1.0;

    pointcloud_count_total_ = 0;
    tracked_count_total_ = 0;
    safety_count_total_ = 0;
    raw_cmd_count_total_ = 0;
    final_cmd_count_total_ = 0;

    count_emergency_ = 0;
    count_hazard_ = 0;
    count_free_ = 0;
    signal_transitions_ = 0;
    override_events_ = 0;

    current_signal_ = 2;
    have_current_signal_ = false;

    current_track_count_ = 0;

    raw_speed_mps_ = 0.0;
    final_speed_mps_ = 0.0;

    have_raw_cmd_ = false;
    have_final_cmd_ = false;

    waiting_for_stop_after_emergency_ = false;
    emergency_request_time_sec_ = -1.0;

    have_cpu_sample_ = false;
    last_cpu_idle_ = 0;
    last_cpu_total_ = 0;
  }

  // ==========================================================================
  // Topic callbacks
  // ==========================================================================
  bool shouldAcceptMessages() const
  {
    return monitoring_active_ && bag_currently_running_;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!shouldAcceptMessages()) {
      return;
    }

    const double now = nowSec();

    if (last_pointcloud_time_sec_ > 0.0) {
      pointcloud_period_ms_.add(durationMs(now, last_pointcloud_time_sec_));
    }

    last_pointcloud_time_sec_ = now;
    pointcloud_count_total_++;

    const rclcpp::Time stamp(msg->header.stamp);

    if (stamp.nanoseconds() > 0) {
      const double age_ms =
        (this->now() - stamp).seconds() * 1000.0;

      pointcloud_age_ms_.add(age_ms);
    }
  }

  void trackedObjectsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!shouldAcceptMessages()) {
      return;
    }

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

      const double source_time_sec =
        static_cast<double>(msg->data[k + 16]);

      const double tracker_processing_ms =
        static_cast<double>(msg->data[k + 17]);

      const double detector_processing_ms =
        static_cast<double>(msg->data[k + 18]);

      if (source_time_sec > 0.0) {
        const double age_ms =
          (nowSec() - source_time_sec) * 1000.0;

        // Only accept tracked age if source_time_sec is in the same time domain.
        // This prevents old rosbag / relative timestamps from creating huge values.
        if (isReasonableTrackedAgeMs(age_ms)) {
          tracked_age_ms_.add(age_ms);
        }
      }

      if (std::isfinite(tracker_processing_ms) &&
          tracker_processing_ms >= 0.0)
      {
        tracker_processing_ms_.add(tracker_processing_ms);
      }

      if (std::isfinite(detector_processing_ms) &&
          detector_processing_ms >= 0.0)
      {
        detector_processing_ms_.add(detector_processing_ms);
      }
    }
  }

  void safetySignalCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    if (!shouldAcceptMessages()) {
      return;
    }

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
    if (!shouldAcceptMessages()) {
      return;
    }

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
    if (!shouldAcceptMessages()) {
      return;
    }

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
  // CPU / memory
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
  // Summary / output
  // ==========================================================================
  void writeCsvRow(
    double pc_hz,
    double tracked_hz,
    double safety_hz,
    double raw_cmd_hz,
    double final_cmd_hz,
    double system_cpu_percent,
    double own_rss_mb)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_
      << fmt(elapsedMonitorTimeSec(), 6) << ","
      << rosbag_base_name_ << ","
      << fmt(pc_hz) << ","
      << fmt(pointcloud_age_ms_.mean()) << ","
      << fmt(tracked_hz) << ","
      << fmt(tracked_age_ms_.mean()) << ","
      << current_track_count_ << ","
      << fmt(detector_processing_ms_.mean()) << ","
      << fmt(tracker_processing_ms_.mean()) << ","
      << fmt(safety_hz) << ","
      << current_signal_ << ","
      << count_emergency_ << ","
      << count_hazard_ << ","
      << count_free_ << ","
      << signal_transitions_ << ","
      << fmt(raw_cmd_hz) << ","
      << fmt(final_cmd_hz) << ","
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

  void writeTxtSummary(
    double pc_hz,
    double tracked_hz,
    double safety_hz,
    double raw_cmd_hz,
    double final_cmd_hz,
    double system_cpu_percent,
    double own_rss_mb)
  {
    if (!txt_.is_open()) {
      return;
    }

    const std::string timestamp =
      formatLocalTime(std::chrono::system_clock::now(), "%H:%M:%S");

    txt_ << "time: " << timestamp << "\n";
    txt_ << "pointcloud_hz: " << fmt(pc_hz) << "\n";
    txt_ << "pointcloud_age_mean_ms: " << fmt(pointcloud_age_ms_.mean()) << "\n";
    txt_ << "tracked_hz: " << fmt(tracked_hz) << "\n";
    txt_ << "tracked_age_mean_ms: " << fmt(tracked_age_ms_.mean()) << "\n";
    txt_ << "track_count: " << current_track_count_ << "\n";
    txt_ << "detector_processing_mean_ms: " << fmt(detector_processing_ms_.mean()) << "\n";
    txt_ << "tracker_processing_mean_ms: " << fmt(tracker_processing_ms_.mean()) << "\n";
    txt_ << "safety_hz: " << fmt(safety_hz) << "\n";
    txt_ << "current_signal: " << current_signal_ << "\n";
    txt_ << "count_emergency: " << count_emergency_ << "\n";
    txt_ << "count_hazard: " << count_hazard_ << "\n";
    txt_ << "count_free: " << count_free_ << "\n";
    txt_ << "signal_transitions: " << signal_transitions_ << "\n";
    txt_ << "raw_cmd_hz: " << fmt(raw_cmd_hz) << "\n";
    txt_ << "final_cmd_hz: " << fmt(final_cmd_hz) << "\n";
    txt_ << "raw_speed_mps: " << fmt(raw_speed_mps_) << "\n";
    txt_ << "final_speed_mps: " << fmt(final_speed_mps_) << "\n";
    txt_ << "override_events: " << override_events_ << "\n";
    txt_ << "reaction_mean_ms: " << fmt(reaction_time_ms_.mean()) << "\n";
    txt_ << "reaction_min_ms: " << fmt(reaction_time_ms_.min()) << "\n";
    txt_ << "reaction_max_ms: " << fmt(reaction_time_ms_.max()) << "\n";
    txt_ << "system_cpu_percent: " << fmt(system_cpu_percent) << "\n";
    txt_ << "monitor_rss_mb: " << fmt(own_rss_mb) << "\n";
    txt_ << "----------------------\n";

    txt_.flush();
  }

  void publishSummary(
    double pc_hz,
    double tracked_hz,
    double safety_hz,
    double system_cpu_percent,
    double own_rss_mb)
  {
    std::ostringstream ss;

    ss
      << "Safety monitor summary\n"
      << "----------------------\n"
      << "rosbag_name: " << rosbag_base_name_ << "\n"
      << "time_sec: " << fmt(elapsedMonitorTimeSec()) << "\n"
      << "pointcloud_hz: " << fmt(pc_hz) << "\n"
      << "tracked_hz: " << fmt(tracked_hz) << "\n"
      << "track_count: " << current_track_count_ << "\n"
      << "safety_hz: " << fmt(safety_hz) << "\n"
      << "current_signal: " << current_signal_ << "\n"
      << "count_emergency: " << count_emergency_ << "\n"
      << "count_hazard: " << count_hazard_ << "\n"
      << "count_free: " << count_free_ << "\n"
      << "signal_transitions: " << signal_transitions_ << "\n"
      << "reaction_mean_ms: " << fmt(reaction_time_ms_.mean()) << "\n"
      << "system_cpu_percent: " << fmt(system_cpu_percent) << "\n"
      << "monitor_rss_mb: " << fmt(own_rss_mb) << "\n";

    std_msgs::msg::String out;
    out.data = ss.str();
    summary_pub_->publish(out);
  }

  // ==========================================================================
  // Timer
  // ==========================================================================
  void timerCallback()
  {
    const RosbagProcessInfo bag = findActiveRosbagProcess();

    if (!monitoring_active_) {
      if (!bag.running) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Waiting for active ros2 bag play process...");
        return;
      }

      startMonitoringForBag(bag);

      if (!monitoring_active_) {
        return;
      }
    }

    if (monitoring_active_) {
      if (!bag.running) {
        bag_currently_running_ = false;
        missing_bag_cycles_++;

        RCLCPP_INFO(
          this->get_logger(),
          "Rosbag process no longer detected. Missing cycle %d/%d",
          missing_bag_cycles_,
          bag_missing_cycles_before_stop_);

        if (missing_bag_cycles_ >= bag_missing_cycles_before_stop_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Rosbag finished. Shutting down performance monitor.");
          rclcpp::shutdown();
          return;
        }
      } else {
        bag_currently_running_ = true;
        missing_bag_cycles_ = 0;
      }
    }

    if (!monitoring_active_ || !bag_currently_running_) {
      return;
    }

    const double system_cpu_percent = updateSystemCpuPercent();
    const double own_rss_mb = getOwnRssMb();

    const double pc_hz = safeHzFromPeriodMs(pointcloud_period_ms_.mean());
    const double tracked_hz = safeHzFromPeriodMs(tracked_period_ms_.mean());
    const double safety_hz = safeHzFromPeriodMs(safety_period_ms_.mean());
    const double raw_cmd_hz = safeHzFromPeriodMs(raw_cmd_period_ms_.mean());
    const double final_cmd_hz = safeHzFromPeriodMs(final_cmd_period_ms_.mean());

    publishSummary(
      pc_hz,
      tracked_hz,
      safety_hz,
      system_cpu_percent,
      own_rss_mb);

    writeCsvRow(
      pc_hz,
      tracked_hz,
      safety_hz,
      raw_cmd_hz,
      final_cmd_hz,
      system_cpu_percent,
      own_rss_mb);

    if (enable_txt_output_) {
      writeTxtSummary(
        pc_hz,
        tracked_hz,
        safety_hz,
        raw_cmd_hz,
        final_cmd_hz,
        system_cpu_percent,
        own_rss_mb);
    }

    if (debug_) {
      RCLCPP_INFO(
        this->get_logger(),
        "bag=%s time=%s pc_hz=%s tracked_hz=%s safety_hz=%s signal=%d tracks=%d reaction_mean_ms=%s cpu=%s rss=%s",
        rosbag_base_name_.c_str(),
        fmt(elapsedMonitorTimeSec()).c_str(),
        fmt(pc_hz).c_str(),
        fmt(tracked_hz).c_str(),
        fmt(safety_hz).c_str(),
        current_signal_,
        current_track_count_,
        fmt(reaction_time_ms_.mean()).c_str(),
        fmt(system_cpu_percent).c_str(),
        fmt(own_rss_mb).c_str());
    }
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
  std::string output_dir_;

  double summary_period_sec_ = 1.0;
  double stop_speed_threshold_mps_ = 0.03;
  double override_speed_epsilon_mps_ = 0.05;
  double max_tracked_age_ms_ = 5000.0;

  int bag_missing_cycles_before_stop_ = 3;

  // ==========================================================================
  // Rosbag/output state
  // ==========================================================================
  bool monitoring_active_ = false;
  bool bag_currently_running_ = false;
  int missing_bag_cycles_ = 0;

  std::string rosbag_base_name_;
  std::string start_date_str_;
  std::string start_time_str_;

  std::chrono::steady_clock::time_point monitor_start_wall_time_;
  bool monitor_start_wall_time_set_ = false;

  std::string csv_running_path_;
  std::string txt_running_path_;

  std::ofstream csv_;
  std::ofstream txt_;

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
  // Counters/state
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
  // Debug / modes
  // ==========================================================================
  bool debug_ = true;
  bool enable_txt_output_ = false;
  bool enable_sim_time_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyPerformanceMonitor>());
  rclcpp::shutdown();
  return 0;
}