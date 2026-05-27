#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

struct TrackInput
{
  int id = -1;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float vx = 0.0f;
  float vy = 0.0f;
  float vz = 0.0f;

  float closing_speed = 0.0f;

  float dx = 0.0f;
  float dy = 0.0f;
  float dz = 0.0f;

  float distance_xy = 0.0f;

  int num_points = 0;
  int hits = 0;
  int misses = 0;
  int age = 0;

  double source_time_sec = 0.0;
  double tracker_processing_ms = 0.0;
  double detector_processing_ms = 0.0;
};

class SafetySupervisor : public rclcpp::Node
{
public:
  SafetySupervisor()
  : Node("safety_supervisor")
  {
    declareParams();
    loadParams();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)))
                 .reliable()
                 .durability_volatile();

    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_,
      qos,
      std::bind(&SafetySupervisor::tracksCallback, this, _1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      ego_speed_topic_,
      10,
      std::bind(&SafetySupervisor::speedCallback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Int16>(output_topic_, 10);

    // RViz-friendly: keep latest zone markers available for late subscribers
    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/safety_zones",
      marker_qos);

    const double period = 1.0 / std::max(1.0, publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&SafetySupervisor::timerCallback, this));

    last_tracks_time_ = this->now();
    last_speed_time_ = this->now();
    emergency_until_ = this->now();
    hazard_until_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SafetySupervisor started");
    RCLCPP_INFO(this->get_logger(), "tracked_objects topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "ego_speed topic:      %s", ego_speed_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "safety_signal topic:  %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "marker topic:         /safety_zones");
  }

private:
  // =========================================================
  // PARAMETERS
  // =========================================================
  void declareParams()
  {
    // Topics
    this->declare_parameter<std::string>("input_topic", "/tracked_objects");
    this->declare_parameter<std::string>("output_topic", "/safety_signal");
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");

    // QoS / timing
    this->declare_parameter<int>("qos_depth", 20);
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    // Fail-safe
    this->declare_parameter<double>("stale_timeout_sec", 0.5);
    this->declare_parameter<int>("stale_signal", 0);
    this->declare_parameter<double>("ego_speed_stale_timeout_sec", 1.0);

    // Sensor offset model (vehicle frame x=0 at front bumper)
    this->declare_parameter<double>("sensor_offset_x", -0.5);
    this->declare_parameter<double>("sensor_offset_y", 0.0);
    this->declare_parameter<double>("sensor_offset_z", 1.8);

    // Front corridor / ROI in vehicle-front frame
    this->declare_parameter<double>("corridor_half_width", 1.0);
    this->declare_parameter<double>("min_x_consider", 0.0);
    this->declare_parameter<double>("max_x_consider", 20.0);

    // Front distance thresholds
    this->declare_parameter<double>("emergency_distance", 1.5);
    this->declare_parameter<double>("hazard_distance", 2.5);

    // TTC
    this->declare_parameter<bool>("enable_ttc", true);
    this->declare_parameter<double>("emergency_ttc", 0.6);
    this->declare_parameter<double>("hazard_ttc", 1.5);
    this->declare_parameter<double>("min_closing_speed", 0.10);

    // Track filtering
    this->declare_parameter<int>("min_track_hits", 1);
    this->declare_parameter<int>("max_track_misses", 2);

    // Cut-in
    this->declare_parameter<bool>("enable_cut_in_prediction", true);
    this->declare_parameter<double>("hazard_prediction_horizon", 1.5);
    this->declare_parameter<double>("emergency_prediction_horizon", 0.5);
    this->declare_parameter<double>("min_lateral_speed", 0.10);

    // Hold times
    this->declare_parameter<double>("emergency_hold_sec", 0.30);
    this->declare_parameter<double>("hazard_hold_sec", 0.50);

    // Brake model
    this->declare_parameter<bool>("enable_brake_model", true);
    this->declare_parameter<double>("ego_speed_mps", 1.3889);
    this->declare_parameter<double>("max_decel_mps2", 1.5);
    this->declare_parameter<double>("system_delay_sec", 0.30);
    this->declare_parameter<double>("emergency_margin_m", 0.10);
    this->declare_parameter<double>("hazard_margin_m", 0.25);

    // Side zones
    this->declare_parameter<bool>("enable_side_zones", true);
    this->declare_parameter<double>("side_zone_length", 6.0);
    this->declare_parameter<double>("side_zone_width", 0.6);
    this->declare_parameter<double>("side_zone_offset_y", 1.2);
    this->declare_parameter<double>("min_side_approach_speed", 0.10);

    // Hysteresis
    this->declare_parameter<double>("emergency_distance_hysteresis", 0.30);
    this->declare_parameter<double>("hazard_distance_hysteresis", 0.30);
    this->declare_parameter<double>("emergency_ttc_hysteresis", 0.20);
    this->declare_parameter<double>("hazard_ttc_hysteresis", 0.20);
    this->declare_parameter<double>("side_zone_hysteresis_x", 0.20);
    this->declare_parameter<double>("side_zone_hysteresis_y", 0.10);

    // Visualization
    this->declare_parameter<std::string>("marker_frame_id", "velodyne");
    this->declare_parameter<double>("zone_height", 2.0);

    // Debug
    this->declare_parameter<bool>("debug", true);
    this->declare_parameter<int>("debug_every_n_frames", 1);
    this->declare_parameter<bool>("debug_timer", true);
  }

  void loadParams()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    ego_speed_topic_ = this->get_parameter("ego_speed_topic").as_string();

    qos_depth_ = this->get_parameter("qos_depth").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    stale_timeout_sec_ = this->get_parameter("stale_timeout_sec").as_double();
    stale_signal_ = this->get_parameter("stale_signal").as_int();
    ego_speed_stale_timeout_sec_ = this->get_parameter("ego_speed_stale_timeout_sec").as_double();

    sensor_offset_x_ = this->get_parameter("sensor_offset_x").as_double();
    sensor_offset_y_ = this->get_parameter("sensor_offset_y").as_double();
    sensor_offset_z_ = this->get_parameter("sensor_offset_z").as_double();

    corridor_half_width_ = this->get_parameter("corridor_half_width").as_double();
    min_x_consider_ = this->get_parameter("min_x_consider").as_double();
    max_x_consider_ = this->get_parameter("max_x_consider").as_double();

    emergency_distance_ = this->get_parameter("emergency_distance").as_double();
    hazard_distance_ = this->get_parameter("hazard_distance").as_double();

    enable_ttc_ = this->get_parameter("enable_ttc").as_bool();
    emergency_ttc_ = this->get_parameter("emergency_ttc").as_double();
    hazard_ttc_ = this->get_parameter("hazard_ttc").as_double();
    min_closing_speed_ = this->get_parameter("min_closing_speed").as_double();

    min_track_hits_ = this->get_parameter("min_track_hits").as_int();
    max_track_misses_ = this->get_parameter("max_track_misses").as_int();

    enable_cut_in_prediction_ = this->get_parameter("enable_cut_in_prediction").as_bool();
    hazard_prediction_horizon_ = this->get_parameter("hazard_prediction_horizon").as_double();
    emergency_prediction_horizon_ = this->get_parameter("emergency_prediction_horizon").as_double();
    min_lateral_speed_ = this->get_parameter("min_lateral_speed").as_double();

    emergency_hold_sec_ = this->get_parameter("emergency_hold_sec").as_double();
    hazard_hold_sec_ = this->get_parameter("hazard_hold_sec").as_double();

    enable_brake_model_ = this->get_parameter("enable_brake_model").as_bool();
    ego_speed_fallback_mps_ = this->get_parameter("ego_speed_mps").as_double();
    max_decel_mps2_ = this->get_parameter("max_decel_mps2").as_double();
    system_delay_sec_ = this->get_parameter("system_delay_sec").as_double();
    emergency_margin_m_ = this->get_parameter("emergency_margin_m").as_double();
    hazard_margin_m_ = this->get_parameter("hazard_margin_m").as_double();

    enable_side_zones_ = this->get_parameter("enable_side_zones").as_bool();
    side_zone_length_ = this->get_parameter("side_zone_length").as_double();
    side_zone_width_ = this->get_parameter("side_zone_width").as_double();
    side_zone_offset_y_ = this->get_parameter("side_zone_offset_y").as_double();
    min_side_approach_speed_ = this->get_parameter("min_side_approach_speed").as_double();

    emergency_distance_hysteresis_ = this->get_parameter("emergency_distance_hysteresis").as_double();
    hazard_distance_hysteresis_ = this->get_parameter("hazard_distance_hysteresis").as_double();
    emergency_ttc_hysteresis_ = this->get_parameter("emergency_ttc_hysteresis").as_double();
    hazard_ttc_hysteresis_ = this->get_parameter("hazard_ttc_hysteresis").as_double();
    side_zone_hysteresis_x_ = this->get_parameter("side_zone_hysteresis_x").as_double();
    side_zone_hysteresis_y_ = this->get_parameter("side_zone_hysteresis_y").as_double();

    marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();
    zone_height_ = this->get_parameter("zone_height").as_double();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ = this->get_parameter("debug_every_n_frames").as_int();
    debug_timer_ = this->get_parameter("debug_timer").as_bool();

    // Sanity
    if (qos_depth_ < 1) qos_depth_ = 1;
    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 20.0;

    stale_timeout_sec_ = std::max(0.0, stale_timeout_sec_);
    ego_speed_stale_timeout_sec_ = std::max(0.0, ego_speed_stale_timeout_sec_);

    corridor_half_width_ = std::max(0.0, corridor_half_width_);
    if (max_x_consider_ < min_x_consider_) max_x_consider_ = min_x_consider_;

    if (hazard_distance_ < emergency_distance_) hazard_distance_ = emergency_distance_;
    if (hazard_ttc_ < emergency_ttc_) hazard_ttc_ = emergency_ttc_;

    min_closing_speed_ = std::max(0.0, min_closing_speed_);
    min_track_hits_ = std::max(1, min_track_hits_);
    max_track_misses_ = std::max(0, max_track_misses_);

    hazard_prediction_horizon_ = std::max(0.0, hazard_prediction_horizon_);
    emergency_prediction_horizon_ = std::max(0.0, emergency_prediction_horizon_);
    min_lateral_speed_ = std::max(0.0, min_lateral_speed_);

    emergency_hold_sec_ = std::max(0.0, emergency_hold_sec_);
    hazard_hold_sec_ = std::max(0.0, hazard_hold_sec_);

    ego_speed_fallback_mps_ = std::max(0.0, ego_speed_fallback_mps_);
    if (max_decel_mps2_ <= 0.0) max_decel_mps2_ = 1.0;
    system_delay_sec_ = std::max(0.0, system_delay_sec_);
    emergency_margin_m_ = std::max(0.0, emergency_margin_m_);
    hazard_margin_m_ = std::max(0.0, hazard_margin_m_);

    side_zone_length_ = std::max(0.0, side_zone_length_);
    side_zone_width_ = std::max(0.0, side_zone_width_);
    side_zone_offset_y_ = std::max(0.0, side_zone_offset_y_);
    min_side_approach_speed_ = std::max(0.0, min_side_approach_speed_);

    emergency_distance_hysteresis_ = std::max(0.0, emergency_distance_hysteresis_);
    hazard_distance_hysteresis_ = std::max(0.0, hazard_distance_hysteresis_);
    emergency_ttc_hysteresis_ = std::max(0.0, emergency_ttc_hysteresis_);
    hazard_ttc_hysteresis_ = std::max(0.0, hazard_ttc_hysteresis_);
    side_zone_hysteresis_x_ = std::max(0.0, side_zone_hysteresis_x_);
    side_zone_hysteresis_y_ = std::max(0.0, side_zone_hysteresis_y_);

    zone_height_ = std::max(0.1, zone_height_);

    if (debug_every_n_frames_ < 1) debug_every_n_frames_ = 1;
  }

  // =========================================================
  // INPUT CALLBACKS
  // =========================================================
  void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    ego_speed_live_mps_ = std::max(0.0f, msg->data);
    last_speed_time_ = this->now();
    have_ego_speed_ = true;
  }

  void tracksCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    ++frame_count_;
    last_tracks_time_ = this->now();
    tracks_ = parseTracks(*msg);
  }

  // =========================================================
  // PARSE TRACKS
  // =========================================================
  std::vector<TrackInput> parseTracks(const std_msgs::msg::Float32MultiArray & msg)
  {
    std::vector<TrackInput> tracks;
    if (msg.data.empty()) return tracks;

    std::size_t stride = 0;
    if (msg.layout.dim.size() >= 2 && msg.layout.dim[1].size > 0) {
      stride = msg.layout.dim[1].size;
    }

    if (stride != 13 && stride != 19) {
      if (msg.data.size() % 13 == 0) stride = 13;
      else if (msg.data.size() % 19 == 0) stride = 19;
      else return tracks;
    }

    const std::size_t n = msg.data.size() / stride;
    tracks.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t k = i * stride;

      TrackInput t;
      t.id = static_cast<int>(msg.data[k + 0]);
      t.x = msg.data[k + 1];
      t.y = msg.data[k + 2];
      t.z = msg.data[k + 3];
      t.vx = msg.data[k + 4];
      t.vy = msg.data[k + 5];
      t.vz = msg.data[k + 6];

      if (stride == 13) {
        t.dx = msg.data[k + 7];
        t.dy = msg.data[k + 8];
        t.dz = msg.data[k + 9];
        t.num_points = static_cast<int>(msg.data[k + 10]);
        t.hits = static_cast<int>(msg.data[k + 11]);
        t.misses = static_cast<int>(msg.data[k + 12]);
        t.distance_xy = static_cast<float>(std::hypot(t.x, t.y));
        t.closing_speed = std::max(0.0f, -t.vx);
        t.age = t.hits + t.misses;
      } else {
        t.closing_speed = msg.data[k + 7];
        t.dx = msg.data[k + 8];
        t.dy = msg.data[k + 9];
        t.dz = msg.data[k + 10];
        t.distance_xy = msg.data[k + 11];
        t.num_points = static_cast<int>(msg.data[k + 12]);
        t.hits = static_cast<int>(msg.data[k + 13]);
        t.misses = static_cast<int>(msg.data[k + 14]);
        t.age = static_cast<int>(msg.data[k + 15]);
        t.source_time_sec = static_cast<double>(msg.data[k + 16]);
        t.tracker_processing_ms = static_cast<double>(msg.data[k + 17]);
        t.detector_processing_ms = static_cast<double>(msg.data[k + 18]);
      }

      tracks.push_back(t);
    }

    return tracks;
  }

  // =========================================================
  // HELPERS
  // =========================================================
  double getCurrentEgoSpeed() const
  {
    if (have_ego_speed_) {
      const double age = (this->now() - last_speed_time_).seconds();
      if (age <= ego_speed_stale_timeout_sec_) {
        return static_cast<double>(ego_speed_live_mps_);
      }
    }
    return ego_speed_fallback_mps_;
  }

  void toVehicleFrame(const TrackInput & t, double & x_v, double & y_v, double & z_v) const
  {
    x_v = static_cast<double>(t.x) + sensor_offset_x_;
    y_v = static_cast<double>(t.y) + sensor_offset_y_;
    z_v = static_cast<double>(t.z) + sensor_offset_z_;
  }

  double vehicleXToSensorX(double x_vehicle) const
  {
    return x_vehicle - sensor_offset_x_;
  }

  double stoppingDistance(double v, double a, double delay, double margin) const
  {
    if (a <= 0.0) return std::numeric_limits<double>::infinity();
    return (v * v) / (2.0 * a) + v * delay + margin;
  }

  bool willEnterCorridor(
    double y_v,
    double vy,
    double half_y,
    double prediction_horizon,
    double & time_to_corridor) const
  {
    const double lateral_limit = corridor_half_width_ + half_y;
    time_to_corridor = std::numeric_limits<double>::infinity();

    if (std::abs(y_v) <= lateral_limit) {
      time_to_corridor = 0.0;
      return true;
    }

    if (std::abs(vy) < min_lateral_speed_) return false;

    if (y_v > lateral_limit && vy >= 0.0) return false;
    if (y_v < -lateral_limit && vy <= 0.0) return false;

    const double distance_to_corridor = std::abs(y_v) - lateral_limit;
    time_to_corridor = distance_to_corridor / std::abs(vy);

    return time_to_corridor <= prediction_horizon;
  }

  // Side total band is split into inner EMERGENCY half and outer HAZARD half
  bool inLeftSideEmergencyApproaching(double x_v, double y_v, double vy, bool latched) const
  {
    const double extra_x = latched ? side_zone_hysteresis_x_ : 0.0;
    const double extra_y = latched ? side_zone_hysteresis_y_ : 0.0;

    const double x_min = -side_zone_length_ - extra_x;
    const double x_max = 0.0 + extra_x;

    const double total_half = side_zone_width_ / 2.0 + extra_y;
    const double y_inner = side_zone_offset_y_ - total_half;  // nearest centerline
    const double y_mid = side_zone_offset_y_;

    const bool inside = (x_v >= x_min && x_v <= x_max && y_v >= y_inner && y_v <= y_mid);
    const bool approaching = (vy <= -min_side_approach_speed_);
    return inside && approaching;
  }

  bool inLeftSideHazardApproaching(double x_v, double y_v, double vy, bool latched) const
  {
    const double extra_x = latched ? side_zone_hysteresis_x_ : 0.0;
    const double extra_y = latched ? side_zone_hysteresis_y_ : 0.0;

    const double x_min = -side_zone_length_ - extra_x;
    const double x_max = 0.0 + extra_x;

    const double total_half = side_zone_width_ / 2.0 + extra_y;
    const double y_mid = side_zone_offset_y_;
    const double y_outer = side_zone_offset_y_ + total_half;

    const bool inside = (x_v >= x_min && x_v <= x_max && y_v >= y_mid && y_v <= y_outer);
    const bool approaching = (vy <= -min_side_approach_speed_);
    return inside && approaching;
  }

  bool inRightSideEmergencyApproaching(double x_v, double y_v, double vy, bool latched) const
  {
    const double extra_x = latched ? side_zone_hysteresis_x_ : 0.0;
    const double extra_y = latched ? side_zone_hysteresis_y_ : 0.0;

    const double x_min = -side_zone_length_ - extra_x;
    const double x_max = 0.0 + extra_x;

    const double total_half = side_zone_width_ / 2.0 + extra_y;
    const double y_mid = -side_zone_offset_y_;
    const double y_inner = -(side_zone_offset_y_ - total_half);  // nearest centerline

    const bool inside = (x_v >= x_min && x_v <= x_max && y_v >= y_mid && y_v <= y_inner);
    const bool approaching = (vy >= +min_side_approach_speed_);
    return inside && approaching;
  }

  bool inRightSideHazardApproaching(double x_v, double y_v, double vy, bool latched) const
  {
    const double extra_x = latched ? side_zone_hysteresis_x_ : 0.0;
    const double extra_y = latched ? side_zone_hysteresis_y_ : 0.0;

    const double x_min = -side_zone_length_ - extra_x;
    const double x_max = 0.0 + extra_x;

    const double total_half = side_zone_width_ / 2.0 + extra_y;
    const double y_outer = -(side_zone_offset_y_ + total_half);
    const double y_mid = -side_zone_offset_y_;

    const bool inside = (x_v >= x_min && x_v <= x_max && y_v >= y_outer && y_v <= y_mid);
    const bool approaching = (vy >= +min_side_approach_speed_);
    return inside && approaching;
  }

  // =========================================================
  // CORE SAFETY LOGIC WITH HYSTERESIS
  // =========================================================
  int computeRawSafetySignal(
    const std::vector<TrackInput> & tracks,
    double & min_front_distance_out)
  {
    min_front_distance_out = std::numeric_limits<double>::infinity();

    const double v_ego = getCurrentEgoSpeed();

    const double d_emerg_stop =
      enable_brake_model_
        ? stoppingDistance(v_ego, max_decel_mps2_, system_delay_sec_, emergency_margin_m_)
        : 0.0;

    const double d_hazard_stop =
      enable_brake_model_
        ? stoppingDistance(v_ego, max_decel_mps2_, system_delay_sec_, hazard_margin_m_)
        : 0.0;

    const bool emergency_latched = (latched_signal_ == 0);
    const bool hazard_latched = (latched_signal_ == 1);

    const double emergency_distance_enter = std::max(emergency_distance_, d_emerg_stop);
    const double hazard_distance_enter = std::max(hazard_distance_, d_hazard_stop);

    const double emergency_distance_apply =
      emergency_latched ? emergency_distance_enter + emergency_distance_hysteresis_
                        : emergency_distance_enter;

    const double hazard_distance_apply =
      hazard_latched ? hazard_distance_enter + hazard_distance_hysteresis_
                     : hazard_distance_enter;

    const double emergency_ttc_apply =
      emergency_latched ? emergency_ttc_ + emergency_ttc_hysteresis_
                        : emergency_ttc_;

    const double hazard_ttc_apply =
      hazard_latched ? hazard_ttc_ + hazard_ttc_hysteresis_
                     : hazard_ttc_;

    int signal = 2;

    for (const auto & t : tracks) {
      if (t.hits < min_track_hits_) continue;
      if (t.misses > max_track_misses_) continue;

      const double half_x = 0.5 * std::max(0.0f, t.dx);
      const double half_y = 0.5 * std::max(0.0f, t.dy);

      double x_v = 0.0, y_v = 0.0, z_v = 0.0;
      toVehicleFrame(t, x_v, y_v, z_v);
      (void)z_v;

      const double object_front_edge = x_v - half_x;
      const double object_rear_edge = x_v + half_x;

      if (object_rear_edge < min_x_consider_) continue;
      if (object_front_edge > max_x_consider_) continue;

      const double front_distance = std::max(0.0, object_front_edge);
      min_front_distance_out = std::min(min_front_distance_out, front_distance);

      const double lateral_limit = corridor_half_width_ + half_y;
      const bool in_corridor = std::abs(y_v) <= lateral_limit;

      // TTC
      double closing_speed = static_cast<double>(t.closing_speed);
      if (!std::isfinite(closing_speed)) closing_speed = 0.0;

      double ttc = std::numeric_limits<double>::infinity();
      if (enable_ttc_ && closing_speed > min_closing_speed_) {
        ttc = front_distance / closing_speed;
      }

      // Cut-in
      double time_to_corridor_hazard = std::numeric_limits<double>::infinity();
      double time_to_corridor_emergency = std::numeric_limits<double>::infinity();

      bool will_enter_hazard_corridor = false;
      bool will_enter_emergency_corridor = false;

      if (enable_cut_in_prediction_) {
        will_enter_hazard_corridor =
          willEnterCorridor(y_v, static_cast<double>(t.vy), half_y,
                            hazard_prediction_horizon_, time_to_corridor_hazard);

        will_enter_emergency_corridor =
          willEnterCorridor(y_v, static_cast<double>(t.vy), half_y,
                            emergency_prediction_horizon_, time_to_corridor_emergency);
      }

      // SIDE decisions
      bool side_emergency = false;
      bool side_hazard = false;
      if (enable_side_zones_) {
        const bool side_lat = (latched_signal_ == 1 || latched_signal_ == 0);

        const bool l_em = inLeftSideEmergencyApproaching(x_v, y_v, static_cast<double>(t.vy), side_lat);
        const bool l_hz = inLeftSideHazardApproaching(x_v, y_v, static_cast<double>(t.vy), side_lat);

        const bool r_em = inRightSideEmergencyApproaching(x_v, y_v, static_cast<double>(t.vy), side_lat);
        const bool r_hz = inRightSideHazardApproaching(x_v, y_v, static_cast<double>(t.vy), side_lat);

        side_emergency = l_em || r_em;
        side_hazard = l_hz || r_hz;
      }

      // FRONT decisions
      const bool emergency_by_distance =
        in_corridor && (front_distance <= emergency_distance_apply);

      const bool emergency_by_ttc =
        in_corridor && enable_ttc_ && (ttc <= emergency_ttc_apply);

      const bool emergency_by_cut_in =
        !in_corridor && will_enter_emergency_corridor && (front_distance <= hazard_distance_apply);

      const bool hazard_by_distance =
        in_corridor && (front_distance <= hazard_distance_apply);

      const bool hazard_by_ttc =
        in_corridor && enable_ttc_ && (ttc <= hazard_ttc_apply);

      const bool hazard_by_cut_in =
        !in_corridor && will_enter_hazard_corridor && (front_distance <= max_x_consider_);

      if (emergency_by_distance || emergency_by_ttc || emergency_by_cut_in || side_emergency) {
        return 0;
      }

      if (hazard_by_distance || hazard_by_ttc || hazard_by_cut_in || side_hazard) {
        signal = 1;
      }
    }

    return signal;
  }

  // =========================================================
  // OUTPUT / TIMER
  // =========================================================
  void timerCallback()
  {
    const double tracks_age = (this->now() - last_tracks_time_).seconds();
    if (tracks_age > stale_timeout_sec_) {
      std_msgs::msg::Int16 stale_msg;
      stale_msg.data = static_cast<int16_t>(stale_signal_);
      pub_->publish(stale_msg);
      publishZones();
      return;
    }

    double min_front_distance = std::numeric_limits<double>::infinity();
    const int raw_signal = computeRawSafetySignal(tracks_, min_front_distance);

    const auto now = this->now();

    if (raw_signal == 0) {
      latched_signal_ = 0;
      emergency_until_ = now + rclcpp::Duration::from_seconds(emergency_hold_sec_);
    } else if (raw_signal == 1) {
      if (latched_signal_ != 0 || now >= emergency_until_) {
        latched_signal_ = 1;
      }
      hazard_until_ = now + rclcpp::Duration::from_seconds(hazard_hold_sec_);
    } else {
      if (latched_signal_ == 0) {
        if (now >= emergency_until_) {
          latched_signal_ = (now < hazard_until_) ? 1 : 2;
        }
      } else if (latched_signal_ == 1) {
        if (now >= hazard_until_) {
          latched_signal_ = 2;
        }
      } else {
        latched_signal_ = 2;
      }
    }

    std_msgs::msg::Int16 out;
    out.data = static_cast<int16_t>(latched_signal_);
    pub_->publish(out);

    if (debug_ && (frame_count_ % debug_every_n_frames_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "raw_signal=%d latched_signal=%d ego_speed=%.3f m/s tracks=%zu min_front=%.2f",
        raw_signal,
        latched_signal_,
        getCurrentEgoSpeed(),
        tracks_.size(),
        std::isfinite(min_front_distance) ? min_front_distance : -1.0);
    }

    publishZones();
  }

  // =========================================================
  // RVIZ ZONES
  // =========================================================
  void publishZones()
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = marker_frame_id_;
    clear.header.stamp.sec = 0;
    clear.header.stamp.nanosec = 0;
    clear.ns = "zones";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    const double h = zone_height_;
    const double z_center = h / 2.0;  // bottom at ground

    // -------------------------
    // FRONT EMERGENCY
    // 0 -> emergency_distance
    // -------------------------
    {
      visualization_msgs::msg::Marker emergency;
      emergency.header.frame_id = marker_frame_id_;
      emergency.header.stamp.sec = 0;
      emergency.header.stamp.nanosec = 0;
      emergency.ns = "zones";
      emergency.id = 2;
      emergency.type = visualization_msgs::msg::Marker::CUBE;
      emergency.action = visualization_msgs::msg::Marker::ADD;

      emergency.pose.position.x = vehicleXToSensorX(emergency_distance_ / 2.0);
      emergency.pose.position.y = 0.0;
      emergency.pose.position.z = z_center;
      emergency.pose.orientation.w = 1.0;

      emergency.scale.x = emergency_distance_;
      emergency.scale.y = corridor_half_width_ * 2.0;
      emergency.scale.z = h;

      emergency.color.r = 1.0f;
      emergency.color.g = 0.0f;
      emergency.color.b = 0.0f;
      emergency.color.a = 0.35f;

      arr.markers.push_back(emergency);
    }

    // -------------------------
    // FRONT HAZARD (no overlap)
    // emergency_distance -> hazard_distance
    // -------------------------
    {
      const double hazard_length = std::max(0.0, hazard_distance_ - emergency_distance_);

      visualization_msgs::msg::Marker hazard;
      hazard.header.frame_id = marker_frame_id_;
      hazard.header.stamp.sec = 0;
      hazard.header.stamp.nanosec = 0;
      hazard.ns = "zones";
      hazard.id = 1;
      hazard.type = visualization_msgs::msg::Marker::CUBE;
      hazard.action = visualization_msgs::msg::Marker::ADD;

      hazard.pose.position.x = vehicleXToSensorX(emergency_distance_ + hazard_length / 2.0);
      hazard.pose.position.y = 0.0;
      hazard.pose.position.z = z_center;
      hazard.pose.orientation.w = 1.0;

      hazard.scale.x = hazard_length;
      hazard.scale.y = corridor_half_width_ * 2.0;
      hazard.scale.z = h;

      hazard.color.r = 1.0f;
      hazard.color.g = 0.5f;
      hazard.color.b = 0.0f;
      hazard.color.a = 0.25f;

      arr.markers.push_back(hazard);
    }

    // -------------------------
    // SIDE ZONES (split, no overlap)
    // x in vehicle frame = [-side_zone_length, 0]
    // -------------------------
    if (enable_side_zones_) {
      const double x_center_vehicle = -side_zone_length_ / 2.0;
      const double x_center_sensor = vehicleXToSensorX(x_center_vehicle);

      const double emergency_w = side_zone_width_ / 2.0;
      const double hazard_w = side_zone_width_ / 2.0;

      // LEFT emergency = nearest centerline half
      {
        visualization_msgs::msg::Marker left_em;
        left_em.header.frame_id = marker_frame_id_;
        left_em.header.stamp.sec = 0;
        left_em.header.stamp.nanosec = 0;
        left_em.ns = "zones";
        left_em.id = 20;
        left_em.type = visualization_msgs::msg::Marker::CUBE;
        left_em.action = visualization_msgs::msg::Marker::ADD;

        left_em.pose.position.x = x_center_sensor;
        left_em.pose.position.y = side_zone_offset_y_ - emergency_w / 2.0;
        left_em.pose.position.z = z_center;
        left_em.pose.orientation.w = 1.0;

        left_em.scale.x = side_zone_length_;
        left_em.scale.y = emergency_w;
        left_em.scale.z = h;

        left_em.color.r = 1.0f;
        left_em.color.g = 0.0f;
        left_em.color.b = 0.0f;
        left_em.color.a = 0.35f;

        arr.markers.push_back(left_em);
      }

      // LEFT hazard = outer half
      {
        visualization_msgs::msg::Marker left_h;
        left_h.header.frame_id = marker_frame_id_;
        left_h.header.stamp.sec = 0;
        left_h.header.stamp.nanosec = 0;
        left_h.ns = "zones";
        left_h.id = 21;
        left_h.type = visualization_msgs::msg::Marker::CUBE;
        left_h.action = visualization_msgs::msg::Marker::ADD;

        left_h.pose.position.x = x_center_sensor;
        left_h.pose.position.y = side_zone_offset_y_ + hazard_w / 2.0;
        left_h.pose.position.z = z_center;
        left_h.pose.orientation.w = 1.0;

        left_h.scale.x = side_zone_length_;
        left_h.scale.y = hazard_w;
        left_h.scale.z = h;

        left_h.color.r = 1.0f;
        left_h.color.g = 0.5f;
        left_h.color.b = 0.0f;
        left_h.color.a = 0.25f;

        arr.markers.push_back(left_h);
      }

      // RIGHT emergency = nearest centerline half
      {
        visualization_msgs::msg::Marker right_em;
        right_em.header.frame_id = marker_frame_id_;
        right_em.header.stamp.sec = 0;
        right_em.header.stamp.nanosec = 0;
        right_em.ns = "zones";
        right_em.id = 30;
        right_em.type = visualization_msgs::msg::Marker::CUBE;
        right_em.action = visualization_msgs::msg::Marker::ADD;

        right_em.pose.position.x = x_center_sensor;
        right_em.pose.position.y = -side_zone_offset_y_ + emergency_w / 2.0;
        right_em.pose.position.z = z_center;
        right_em.pose.orientation.w = 1.0;

        right_em.scale.x = side_zone_length_;
        right_em.scale.y = emergency_w;
        right_em.scale.z = h;

        right_em.color.r = 1.0f;
        right_em.color.g = 0.0f;
        right_em.color.b = 0.0f;
        right_em.color.a = 0.35f;

        arr.markers.push_back(right_em);
      }

      // RIGHT hazard = outer half
      {
        visualization_msgs::msg::Marker right_h;
        right_h.header.frame_id = marker_frame_id_;
        right_h.header.stamp.sec = 0;
        right_h.header.stamp.nanosec = 0;
        right_h.ns = "zones";
        right_h.id = 31;
        right_h.type = visualization_msgs::msg::Marker::CUBE;
        right_h.action = visualization_msgs::msg::Marker::ADD;

        right_h.pose.position.x = x_center_sensor;
        right_h.pose.position.y = -side_zone_offset_y_ - hazard_w / 2.0;
        right_h.pose.position.z = z_center;
        right_h.pose.orientation.w = 1.0;

        right_h.scale.x = side_zone_length_;
        right_h.scale.y = hazard_w;
        right_h.scale.z = h;

        right_h.color.r = 1.0f;
        right_h.color.g = 0.5f;
        right_h.color.b = 0.0f;
        right_h.color.a = 0.25f;

        arr.markers.push_back(right_h);
      }
    }

    marker_pub_->publish(arr);
  }

  // =========================================================
  // MEMBERS
  // =========================================================
  std::string input_topic_;
  std::string output_topic_;
  std::string ego_speed_topic_;
  std::string marker_frame_id_;

  int qos_depth_ = 20;
  double publish_rate_hz_ = 20.0;

  double stale_timeout_sec_ = 0.5;
  int stale_signal_ = 0;
  double ego_speed_stale_timeout_sec_ = 1.0;

  double sensor_offset_x_ = -0.5;
  double sensor_offset_y_ = 0.0;
  double sensor_offset_z_ = 1.8;

  double corridor_half_width_ = 1.0;
  double min_x_consider_ = 0.0;
  double max_x_consider_ = 20.0;

  double emergency_distance_ = 1.5;
  double hazard_distance_ = 2.5;

  bool enable_ttc_ = true;
  double emergency_ttc_ = 0.6;
  double hazard_ttc_ = 1.5;
  double min_closing_speed_ = 0.10;

  int min_track_hits_ = 1;
  int max_track_misses_ = 2;

  bool enable_cut_in_prediction_ = true;
  double hazard_prediction_horizon_ = 1.5;
  double emergency_prediction_horizon_ = 0.5;
  double min_lateral_speed_ = 0.10;

  double emergency_hold_sec_ = 0.30;
  double hazard_hold_sec_ = 0.50;

  bool enable_brake_model_ = true;
  double ego_speed_fallback_mps_ = 1.3889;
  double max_decel_mps2_ = 1.5;
  double system_delay_sec_ = 0.30;
  double emergency_margin_m_ = 0.10;
  double hazard_margin_m_ = 0.25;

  bool enable_side_zones_ = true;
  double side_zone_length_ = 6.0;
  double side_zone_width_ = 0.6;
  double side_zone_offset_y_ = 1.2;
  double min_side_approach_speed_ = 0.10;

  double emergency_distance_hysteresis_ = 0.30;
  double hazard_distance_hysteresis_ = 0.30;
  double emergency_ttc_hysteresis_ = 0.20;
  double hazard_ttc_hysteresis_ = 0.20;
  double side_zone_hysteresis_x_ = 0.20;
  double side_zone_hysteresis_y_ = 0.10;

  double zone_height_ = 2.0;

  bool debug_ = true;
  int debug_every_n_frames_ = 1;
  bool debug_timer_ = true;

  std::vector<TrackInput> tracks_;
  bool have_ego_speed_ = false;
  float ego_speed_live_mps_ = 0.0f;
  int latched_signal_ = 2;   // 0=Emergency, 1=Hazard, 2=Free
  long frame_count_ = 0;

  rclcpp::Time last_tracks_time_;
  rclcpp::Time last_speed_time_;
  rclcpp::Time emergency_until_;
  rclcpp::Time hazard_until_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}