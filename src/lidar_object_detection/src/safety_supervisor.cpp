// safety_supervisor.cpp
// SafetySupervisor node where visualization geometry matches detection geometry.
//
// Detection zones:
// - Front/cab emergency:
//     semicircle sector, angle -90..+90 deg, radius 0.0 -> emergency_distance
// - Front/cab hazard:
//     semicircle ring, angle -90..+90 deg, radius emergency_distance -> hazard_distance
// - Side zones:
//     same rectangular VLP-16 frustum zones as shown in RViz
//
// RViz zones and detection zones are intentionally identical.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
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

enum class ZoneLevel : int
{
  Emergency = 0,
  Hazard = 1,
  Free = 2
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

    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_,
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
    RCLCPP_INFO(this->get_logger(), "marker topic:         %s", marker_topic_.c_str());
  }

private:
  // =========================================================
  // PARAMETERS
  // =========================================================
  void declareParams()
  {
    this->declare_parameter<std::string>("input_topic", "/tracked_objects");
    this->declare_parameter<std::string>("output_topic", "/safety_signal");
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");

    this->declare_parameter<int>("qos_depth", 20);
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    this->declare_parameter<double>("stale_timeout_sec", 0.5);
    this->declare_parameter<int>("stale_signal", 0);
    this->declare_parameter<double>("ego_speed_stale_timeout_sec", 1.0);

    // Vehicle frame:
    // x = 0 at front bumper
    // x positive forward
    // y positive left
    // z positive up from ground
    this->declare_parameter<double>("sensor_offset_x", -0.57);
    this->declare_parameter<double>("sensor_offset_y", 0.0);
    this->declare_parameter<double>("sensor_offset_z", 1.8);

    // Kept for compatibility with older YAML/configs.
    this->declare_parameter<double>("corridor_half_width", 1.0);
    this->declare_parameter<double>("min_x_consider", 0.0);
    this->declare_parameter<double>("max_x_consider", 20.0);

    // Front/cab sector distances.
    this->declare_parameter<double>("emergency_distance", 1.5);
    this->declare_parameter<double>("hazard_distance", 2.5);

    // Kept for compatibility.
    this->declare_parameter<bool>("enable_ttc", true);
    this->declare_parameter<double>("emergency_ttc", 0.6);
    this->declare_parameter<double>("hazard_ttc", 1.5);
    this->declare_parameter<double>("min_closing_speed", 0.10);

    this->declare_parameter<int>("min_track_hits", 1);
    this->declare_parameter<int>("max_track_misses", 2);

    // Kept for compatibility.
    this->declare_parameter<bool>("enable_cut_in_prediction", true);
    this->declare_parameter<double>("hazard_prediction_horizon", 1.5);
    this->declare_parameter<double>("emergency_prediction_horizon", 0.5);
    this->declare_parameter<double>("min_lateral_speed", 0.10);

    this->declare_parameter<double>("emergency_hold_sec", 0.30);
    this->declare_parameter<double>("hazard_hold_sec", 0.50);

    // Kept for compatibility.
    this->declare_parameter<bool>("enable_brake_model", true);
    this->declare_parameter<double>("ego_speed_mps", 1.3889);
    this->declare_parameter<double>("max_decel_mps2", 1.5);
    this->declare_parameter<double>("system_delay_sec", 0.30);
    this->declare_parameter<double>("emergency_margin_m", 0.10);
    this->declare_parameter<double>("hazard_margin_m", 0.25);

    this->declare_parameter<bool>("enable_side_zones", true);
    this->declare_parameter<double>("side_zone_length", 6.0);
    this->declare_parameter<double>("side_zone_width", 0.6);
    this->declare_parameter<double>("side_zone_offset_y", 1.2);
    this->declare_parameter<double>("min_side_approach_speed", 0.10);

    // Kept for compatibility.
    this->declare_parameter<double>("emergency_distance_hysteresis", 0.30);
    this->declare_parameter<double>("hazard_distance_hysteresis", 0.30);
    this->declare_parameter<double>("emergency_ttc_hysteresis", 0.20);
    this->declare_parameter<double>("hazard_ttc_hysteresis", 0.20);
    this->declare_parameter<double>("side_zone_hysteresis_x", 0.20);
    this->declare_parameter<double>("side_zone_hysteresis_y", 0.10);

    this->declare_parameter<std::string>("marker_topic", "/safety_zones_array");
    this->declare_parameter<std::string>("marker_frame_id", "velodyne");

    // Visual cap in vehicle-frame height.
    this->declare_parameter<double>("zone_height", 3.0);

    // VLP-16 vertical FOV model around horizontal sensor line.
    this->declare_parameter<double>("lidar_down_angle_deg", 15.0);
    this->declare_parameter<double>("lidar_up_angle_deg", 15.0);

    this->declare_parameter<int>("sector_azimuth_steps", 120);

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

    marker_topic_ = this->get_parameter("marker_topic").as_string();
    marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();

    zone_height_ = this->get_parameter("zone_height").as_double();

    lidar_down_angle_deg_ = this->get_parameter("lidar_down_angle_deg").as_double();
    lidar_up_angle_deg_ = this->get_parameter("lidar_up_angle_deg").as_double();

    sector_azimuth_steps_ = this->get_parameter("sector_azimuth_steps").as_int();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ = this->get_parameter("debug_every_n_frames").as_int();
    debug_timer_ = this->get_parameter("debug_timer").as_bool();

    // Sanity clamps.
    if (qos_depth_ < 1) qos_depth_ = 1;
    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 20.0;

    stale_timeout_sec_ = std::max(0.0, stale_timeout_sec_);
    ego_speed_stale_timeout_sec_ = std::max(0.0, ego_speed_stale_timeout_sec_);

    corridor_half_width_ = std::max(0.0, corridor_half_width_);

    if (max_x_consider_ < min_x_consider_) {
      max_x_consider_ = min_x_consider_;
    }

    if (hazard_distance_ < emergency_distance_) {
      hazard_distance_ = emergency_distance_;
    }

    min_closing_speed_ = std::max(0.0, min_closing_speed_);
    min_track_hits_ = std::max(1, min_track_hits_);
    max_track_misses_ = std::max(0, max_track_misses_);

    hazard_prediction_horizon_ = std::max(0.0, hazard_prediction_horizon_);
    emergency_prediction_horizon_ = std::max(0.0, emergency_prediction_horizon_);
    min_lateral_speed_ = std::max(0.0, min_lateral_speed_);

    emergency_hold_sec_ = std::max(0.0, emergency_hold_sec_);
    hazard_hold_sec_ = std::max(0.0, hazard_hold_sec_);

    ego_speed_fallback_mps_ = std::max(0.0, ego_speed_fallback_mps_);

    if (max_decel_mps2_ <= 0.0) {
      max_decel_mps2_ = 1.0;
    }

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
    lidar_down_angle_deg_ = std::clamp(lidar_down_angle_deg_, 0.1, 89.0);
    lidar_up_angle_deg_ = std::clamp(lidar_up_angle_deg_, 0.1, 89.0);

    if (sector_azimuth_steps_ < 6) {
      sector_azimuth_steps_ = 6;
    }

    if (debug_every_n_frames_ < 1) {
      debug_every_n_frames_ = 1;
    }
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

    if (msg.data.empty()) {
      return tracks;
    }

    std::size_t stride = 0;

    if (msg.layout.dim.size() >= 2 && msg.layout.dim[1].size > 0) {
      stride = msg.layout.dim[1].size;
    }

    if (stride != 13 && stride != 19) {
      if (msg.data.size() % 13 == 0) {
        stride = 13;
      } else if (msg.data.size() % 19 == 0) {
        stride = 19;
      } else {
        return tracks;
      }
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
  // FRAME / MATH HELPERS
  // =========================================================
  double degToRad(double deg) const
  {
    constexpr double pi = 3.14159265358979323846;
    return deg * pi / 180.0;
  }

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

  double vehicleYToSensorY(double y_vehicle) const
  {
    return y_vehicle - sensor_offset_y_;
  }

  double vehicleZToSensorZ(double z_vehicle) const
  {
    return z_vehicle - sensor_offset_z_;
  }

  // Kept for compatibility / future use.
  double stoppingDistance(double v, double a, double delay, double margin) const
  {
    if (a <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    return (v * v) / (2.0 * a) + v * delay + margin;
  }

  // =========================================================
  // VLP-16 VISIBILITY HELPERS
  // =========================================================
  double horizontalRangeFromSensor(double x_vehicle, double y_vehicle) const
  {
    const double dx = x_vehicle - sensor_offset_x_;
    const double dy = y_vehicle - sensor_offset_y_;
    return std::hypot(dx, dy);
  }

  double lowerVisibleHeightAtXY(double x_vehicle, double y_vehicle) const
  {
    const double angle_rad = degToRad(lidar_down_angle_deg_);
    const double range_xy = horizontalRangeFromSensor(x_vehicle, y_vehicle);
    const double z = sensor_offset_z_ - std::tan(angle_rad) * range_xy;

    return std::clamp(z, 0.0, zone_height_);
  }

  double upperVisibleHeightAtXY(double x_vehicle, double y_vehicle) const
  {
    const double angle_rad = degToRad(lidar_up_angle_deg_);
    const double range_xy = horizontalRangeFromSensor(x_vehicle, y_vehicle);
    const double z = sensor_offset_z_ + std::tan(angle_rad) * range_xy;

    return std::clamp(z, 0.0, zone_height_);
  }

  bool verticalIntervalIntersectsVlp16Frustum(
    double x_vehicle,
    double y_vehicle,
    double object_bottom_z,
    double object_top_z) const
  {
    const double low = lowerVisibleHeightAtXY(x_vehicle, y_vehicle);
    const double high = upperVisibleHeightAtXY(x_vehicle, y_vehicle);

    return object_top_z >= low && object_bottom_z <= high;
  }

  // =========================================================
  // DETECTION ZONE CLASSIFICATION
  // This is the same geometry as RViz.
  // =========================================================
  ZoneLevel classifyPointZone(
    double x_vehicle,
    double y_vehicle,
    double object_bottom_z,
    double object_top_z) const
  {
    if (!verticalIntervalIntersectsVlp16Frustum(
          x_vehicle,
          y_vehicle,
          object_bottom_z,
          object_top_z))
    {
      return ZoneLevel::Free;
    }

    // Front/cab semicircle:
    // angle -90 deg -> +90 deg
    // radius 0 -> emergency_distance / hazard_distance
    const double r = std::hypot(x_vehicle, y_vehicle);
    const double angle = std::atan2(y_vehicle, x_vehicle);

    const bool in_front_sector =
      (angle >= -degToRad(90.0)) &&
      (angle <= degToRad(90.0)) &&
      (x_vehicle >= 0.0);

    if (in_front_sector) {
      if (r <= emergency_distance_) {
        return ZoneLevel::Emergency;
      }

      if (r <= hazard_distance_) {
        return ZoneLevel::Hazard;
      }
    }

    // Side/trailer zones.
    if (enable_side_zones_) {
      const double x0 = -side_zone_length_;
      const double x1 = 0.0;

      const bool in_side_x = (x_vehicle >= x0 && x_vehicle <= x1);

      if (in_side_x) {
        const double emergency_w = side_zone_width_ / 2.0;
        const double hazard_w = side_zone_width_ / 2.0;

        const double left_em_y0 = side_zone_offset_y_ - emergency_w;
        const double left_em_y1 = side_zone_offset_y_;
        const double left_hz_y0 = side_zone_offset_y_;
        const double left_hz_y1 = side_zone_offset_y_ + hazard_w;

        const double right_em_y0 = -side_zone_offset_y_;
        const double right_em_y1 = -side_zone_offset_y_ + emergency_w;
        const double right_hz_y0 = -side_zone_offset_y_ - hazard_w;
        const double right_hz_y1 = -side_zone_offset_y_;

        const bool in_left_em =
          y_vehicle >= left_em_y0 && y_vehicle <= left_em_y1;

        const bool in_right_em =
          y_vehicle >= right_em_y0 && y_vehicle <= right_em_y1;

        if (in_left_em || in_right_em) {
          return ZoneLevel::Emergency;
        }

        const bool in_left_hz =
          y_vehicle >= left_hz_y0 && y_vehicle <= left_hz_y1;

        const bool in_right_hz =
          y_vehicle >= right_hz_y0 && y_vehicle <= right_hz_y1;

        if (in_left_hz || in_right_hz) {
          return ZoneLevel::Hazard;
        }
      }
    }

    return ZoneLevel::Free;
  }

  ZoneLevel classifyObjectZone(const TrackInput & t) const
  {
    const double half_x = 0.5 * std::max(0.0f, t.dx);
    const double half_y = 0.5 * std::max(0.0f, t.dy);
    const double half_z = 0.5 * std::max(0.0f, t.dz);

    double x_v = 0.0;
    double y_v = 0.0;
    double z_v = 0.0;

    toVehicleFrame(t, x_v, y_v, z_v);

    const double x_min = x_v - half_x;
    const double x_max = x_v + half_x;
    const double y_min = y_v - half_y;
    const double y_max = y_v + half_y;

    const double object_bottom_z = z_v - half_z;
    const double object_top_z = z_v + half_z;

    std::vector<std::pair<double, double>> samples;
    samples.reserve(6);

    // Center.
    samples.emplace_back(x_v, y_v);

    // Footprint corners.
    samples.emplace_back(x_min, y_min);
    samples.emplace_back(x_min, y_max);
    samples.emplace_back(x_max, y_min);
    samples.emplace_back(x_max, y_max);

    // Closest point of bbox footprint to front/cab origin.
    const double closest_x = std::clamp(0.0, x_min, x_max);
    const double closest_y = std::clamp(0.0, y_min, y_max);
    samples.emplace_back(closest_x, closest_y);

    ZoneLevel best = ZoneLevel::Free;

    for (const auto & s : samples) {
      const ZoneLevel zone =
        classifyPointZone(
          s.first,
          s.second,
          object_bottom_z,
          object_top_z);

      if (static_cast<int>(zone) < static_cast<int>(best)) {
        best = zone;
      }

      if (best == ZoneLevel::Emergency) {
        return best;
      }
    }

    return best;
  }

  // =========================================================
  // VISUALIZATION HELPERS
  // =========================================================
  geometry_msgs::msg::Point makePointVehicle(
    double x_vehicle,
    double y_vehicle,
    double z_vehicle) const
  {
    geometry_msgs::msg::Point p;
    p.x = vehicleXToSensorX(x_vehicle);
    p.y = vehicleYToSensorY(y_vehicle);
    p.z = vehicleZToSensorZ(z_vehicle);

    return p;
  }

  void addTwoSidedTriangle(
    std::vector<geometry_msgs::msg::Point> & pts,
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b,
    const geometry_msgs::msg::Point & c) const
  {
    pts.push_back(a);
    pts.push_back(b);
    pts.push_back(c);

    pts.push_back(c);
    pts.push_back(b);
    pts.push_back(a);
  }

  visualization_msgs::msg::Marker makeVlp16FrustumZoneMarker(
    int id,
    double x0_vehicle,
    double x1_vehicle,
    double y0_vehicle,
    double y1_vehicle,
    float r,
    float g,
    float b,
    float a) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = marker_frame_id_;
    m.header.stamp = this->now();
    m.ns = "zones";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.orientation.w = 1.0;

    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;

    const double zA_low = lowerVisibleHeightAtXY(x0_vehicle, y0_vehicle);
    const double zB_low = lowerVisibleHeightAtXY(x0_vehicle, y1_vehicle);
    const double zC_low = lowerVisibleHeightAtXY(x1_vehicle, y1_vehicle);
    const double zD_low = lowerVisibleHeightAtXY(x1_vehicle, y0_vehicle);

    const double zA_high = upperVisibleHeightAtXY(x0_vehicle, y0_vehicle);
    const double zB_high = upperVisibleHeightAtXY(x0_vehicle, y1_vehicle);
    const double zC_high = upperVisibleHeightAtXY(x1_vehicle, y1_vehicle);
    const double zD_high = upperVisibleHeightAtXY(x1_vehicle, y0_vehicle);

    const auto A = makePointVehicle(x0_vehicle, y0_vehicle, zA_low);
    const auto B = makePointVehicle(x0_vehicle, y1_vehicle, zB_low);
    const auto C = makePointVehicle(x1_vehicle, y1_vehicle, zC_low);
    const auto D = makePointVehicle(x1_vehicle, y0_vehicle, zD_low);

    const auto E = makePointVehicle(x0_vehicle, y0_vehicle, zA_high);
    const auto F = makePointVehicle(x0_vehicle, y1_vehicle, zB_high);
    const auto G = makePointVehicle(x1_vehicle, y1_vehicle, zC_high);
    const auto H = makePointVehicle(x1_vehicle, y0_vehicle, zD_high);

    // Lower surface.
    addTwoSidedTriangle(m.points, A, B, C);
    addTwoSidedTriangle(m.points, A, C, D);

    // Upper surface.
    addTwoSidedTriangle(m.points, E, G, F);
    addTwoSidedTriangle(m.points, E, H, G);

    // x0 wall.
    addTwoSidedTriangle(m.points, A, F, B);
    addTwoSidedTriangle(m.points, A, E, F);

    // x1 wall.
    addTwoSidedTriangle(m.points, D, C, G);
    addTwoSidedTriangle(m.points, D, G, H);

    // y0 wall.
    addTwoSidedTriangle(m.points, A, D, H);
    addTwoSidedTriangle(m.points, A, H, E);

    // y1 wall.
    addTwoSidedTriangle(m.points, B, F, G);
    addTwoSidedTriangle(m.points, B, G, C);

    return m;
  }

  visualization_msgs::msg::Marker makeSectorZoneMarkerClassified(
    int id,
    ZoneLevel target_zone,
    double r_min,
    double r_max,
    double angle_min_rad,
    double angle_max_rad,
    int steps,
    float cr,
    float cg,
    float cb,
    float ca) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = marker_frame_id_;
    m.header.stamp = this->now();
    m.ns = "zones_sector";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.orientation.w = 1.0;

    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;

    m.color.r = cr;
    m.color.g = cg;
    m.color.b = cb;
    m.color.a = ca;

    const int N = std::max(3, steps);
    const double dtheta =
      (angle_max_rad - angle_min_rad) / static_cast<double>(N - 1);

    std::vector<double> thetas(N);

    for (int i = 0; i < N; ++i) {
      thetas[i] = angle_min_rad + static_cast<double>(i) * dtheta;
    }

    struct SamplePoint
    {
      geometry_msgs::msg::Point low;
      geometry_msgs::msg::Point high;
    };

    std::vector<SamplePoint> inner(N);
    std::vector<SamplePoint> outer(N);

    for (int i = 0; i < N; ++i) {
      const double theta = thetas[i];
      const double cos_t = std::cos(theta);
      const double sin_t = std::sin(theta);

      const double xi_inner = r_min * cos_t;
      const double yi_inner = r_min * sin_t;
      const double z_il = lowerVisibleHeightAtXY(xi_inner, yi_inner);
      const double z_ih = upperVisibleHeightAtXY(xi_inner, yi_inner);

      inner[i].low = makePointVehicle(xi_inner, yi_inner, z_il);
      inner[i].high = makePointVehicle(xi_inner, yi_inner, z_ih);

      const double xi_outer = r_max * cos_t;
      const double yi_outer = r_max * sin_t;
      const double z_ol = lowerVisibleHeightAtXY(xi_outer, yi_outer);
      const double z_oh = upperVisibleHeightAtXY(xi_outer, yi_outer);

      outer[i].low = makePointVehicle(xi_outer, yi_outer, z_ol);
      outer[i].high = makePointVehicle(xi_outer, yi_outer, z_oh);
    }

    for (int i = 0; i < N - 1; ++i) {
      const double mid_theta = 0.5 * (thetas[i] + thetas[i + 1]);
      const double mid_r = 0.5 * (r_min + r_max);

      const double mx = mid_r * std::cos(mid_theta);
      const double my = mid_r * std::sin(mid_theta);

      const ZoneLevel mid_zone =
        classifyPointZone(
          mx,
          my,
          0.0,
          zone_height_);

      if (mid_zone != target_zone) {
        continue;
      }

      // Lower surface.
      addTwoSidedTriangle(m.points, inner[i].low, outer[i].low, outer[i + 1].low);
      addTwoSidedTriangle(m.points, inner[i].low, outer[i + 1].low, inner[i + 1].low);

      // Upper surface.
      addTwoSidedTriangle(m.points, inner[i].high, inner[i + 1].high, outer[i + 1].high);
      addTwoSidedTriangle(m.points, inner[i].high, outer[i + 1].high, outer[i].high);

      // Outer wall.
      addTwoSidedTriangle(m.points, outer[i].low, outer[i].high, outer[i + 1].high);
      addTwoSidedTriangle(m.points, outer[i].low, outer[i + 1].high, outer[i + 1].low);

      // Inner wall if r_min > 0.
      if (r_min > 1e-6) {
        addTwoSidedTriangle(m.points, inner[i].low, inner[i + 1].high, inner[i].high);
        addTwoSidedTriangle(m.points, inner[i].low, inner[i + 1].low, inner[i + 1].high);
      }
    }

    return m;
  }

  // =========================================================
  // CORE SAFETY LOGIC
  // =========================================================
  int computeRawSafetySignal(
    const std::vector<TrackInput> & tracks,
    double & min_front_distance_out)
  {
    min_front_distance_out = std::numeric_limits<double>::infinity();

    int signal = 2;

    for (const auto & t : tracks) {
      if (t.hits < min_track_hits_) {
        continue;
      }

      if (t.misses > max_track_misses_) {
        continue;
      }

      const ZoneLevel zone = classifyObjectZone(t);

      if (zone == ZoneLevel::Emergency) {
        return 0;
      }

      if (zone == ZoneLevel::Hazard) {
        signal = 1;
      }

      // Diagnostics only.
      double x_v = 0.0;
      double y_v = 0.0;
      double z_v = 0.0;

      toVehicleFrame(t, x_v, y_v, z_v);

      const double half_x = 0.5 * std::max(0.0f, t.dx);
      const double object_front_edge = x_v - half_x;
      const double front_distance = std::max(0.0, object_front_edge);

      min_front_distance_out =
        std::min(min_front_distance_out, front_distance);
    }

    return signal;
  }

  // =========================================================
  // TIMER / PUBLISH
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
  // PUBLISH ZONES
  // Visualization equals detection geometry.
  // =========================================================
  void publishZones()
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = marker_frame_id_;
    clear.header.stamp = this->now();
    clear.ns = "zones";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    clear.ns = "zones_sector";
    clear.id = 1000;
    arr.markers.push_back(clear);

    const double angle_min = -degToRad(90.0);
    const double angle_max = degToRad(90.0);

    // Front / cab emergency sector.
    arr.markers.push_back(
      makeSectorZoneMarkerClassified(
        2,
        ZoneLevel::Emergency,
        0.0,
        emergency_distance_,
        angle_min,
        angle_max,
        sector_azimuth_steps_,
        1.0f,
        0.0f,
        0.0f,
        0.35f));

    // Front / cab hazard sector.
    arr.markers.push_back(
      makeSectorZoneMarkerClassified(
        1,
        ZoneLevel::Hazard,
        emergency_distance_,
        hazard_distance_,
        angle_min,
        angle_max,
        sector_azimuth_steps_,
        1.0f,
        0.5f,
        0.0f,
        0.25f));

    if (enable_side_zones_) {
      const double x0 = -side_zone_length_;
      const double x1 = 0.0;

      const double emergency_w = side_zone_width_ / 2.0;
      const double hazard_w = side_zone_width_ / 2.0;

      // Left emergency.
      arr.markers.push_back(
        makeVlp16FrustumZoneMarker(
          20,
          x0,
          x1,
          side_zone_offset_y_ - emergency_w,
          side_zone_offset_y_,
          1.0f,
          0.0f,
          0.0f,
          0.35f));

      // Left hazard.
      arr.markers.push_back(
        makeVlp16FrustumZoneMarker(
          21,
          x0,
          x1,
          side_zone_offset_y_,
          side_zone_offset_y_ + hazard_w,
          1.0f,
          0.5f,
          0.0f,
          0.25f));

      // Right emergency.
      arr.markers.push_back(
        makeVlp16FrustumZoneMarker(
          30,
          x0,
          x1,
          -side_zone_offset_y_,
          -side_zone_offset_y_ + emergency_w,
          1.0f,
          0.0f,
          0.0f,
          0.35f));

      // Right hazard.
      arr.markers.push_back(
        makeVlp16FrustumZoneMarker(
          31,
          x0,
          x1,
          -side_zone_offset_y_ - hazard_w,
          -side_zone_offset_y_,
          1.0f,
          0.5f,
          0.0f,
          0.25f));
    }

    marker_pub_->publish(arr);
  }

  // =========================================================
  // MEMBERS
  // =========================================================
  std::string input_topic_;
  std::string output_topic_;
  std::string ego_speed_topic_;
  std::string marker_topic_;
  std::string marker_frame_id_;

  int qos_depth_ = 20;
  double publish_rate_hz_ = 20.0;

  double stale_timeout_sec_ = 0.5;
  int stale_signal_ = 0;
  double ego_speed_stale_timeout_sec_ = 1.0;

  double sensor_offset_x_ = -0.57;
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

  double zone_height_ = 3.0;
  double lidar_down_angle_deg_ = 15.0;
  double lidar_up_angle_deg_ = 15.0;

  int sector_azimuth_steps_ = 120;

  bool debug_ = true;
  int debug_every_n_frames_ = 1;
  bool debug_timer_ = true;

  std::vector<TrackInput> tracks_;
  bool have_ego_speed_ = false;
  float ego_speed_live_mps_ = 0.0f;
  int latched_signal_ = 2;
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