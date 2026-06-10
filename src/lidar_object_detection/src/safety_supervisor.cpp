// ============================================================================
// safety_supervisor.cpp
// ============================================================================
// 3D VLP-16 matched safety supervisor.
//
// Doel:
// - RViz-zones zijn exact dezelfde geometrie als de detectiezones.
// - Front/cab emergency:
//     sector -90°..+90°, radius 0.0 -> emergency_distance
// - Front/cab hazard:
//     sector -90°..+90°, radius emergency_distance -> hazard_distance
// - Side/trailer:
//     left/right side zones zijn ALLEEN hazard.
// - Objecten triggeren alleen als hun hoogte-interval overlapt met het
//   VLP-16 zichtvolume.
// - Marker frame standaard: safety_base.
//
// Frames:
// - Tracks komen binnen in het LiDAR/rosbag-frame: sensor-origin is x=0.
// - sensor_offset_x = -0.57 betekent: sensor staat 0.57 m achter de truckvoorkant.
// - Truckvoorkant ligt dus in het rosbag/sensor-frame op x = +0.57.
// - Front emergency/hazard is 180 graden vanuit de sensor-origin x=0.
// - Side/trailer hazard-zones zijn rechthoeken langs truck/trailer vanaf x=0 naar achteren.
// - RViz-zones en detectiezones gebruiken dezelfde geometrie.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
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

    status_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/safety_status_marker", 1);

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
  // ==========================================================================
  // PARAMETERS
  // ==========================================================================
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

    // Rosbag/LiDAR-frame:
    // sensor-origin is x=0.
    // sensor_offset_x=-0.57 betekent: fysieke truckvoorkant ligt op x=+0.57.
    this->declare_parameter<double>("sensor_offset_x", -0.57);
    this->declare_parameter<double>("sensor_offset_y", 0.0);
    this->declare_parameter<double>("sensor_offset_z", 1.8);

    this->declare_parameter<double>("emergency_distance", 1.5);
    this->declare_parameter<double>("hazard_distance", 2.5);

    this->declare_parameter<bool>("enable_side_zones", true);
    this->declare_parameter<double>("side_zone_length", 2.0);
    this->declare_parameter<double>("side_zone_width", 0.8);
    this->declare_parameter<double>("side_zone_offset_y", 0.8);

    this->declare_parameter<int>("min_track_hits", 1);
    this->declare_parameter<int>("max_track_misses", 2);

    this->declare_parameter<double>("emergency_hold_sec", 0.25);
    this->declare_parameter<double>("hazard_hold_sec", 0.25);

    this->declare_parameter<std::string>("marker_topic", "/safety_zones_array");
    this->declare_parameter<std::string>("marker_frame_id", "safety_base");

    this->declare_parameter<double>("zone_height", 3.0);
    this->declare_parameter<double>("lidar_down_angle_deg", 15.0);
    this->declare_parameter<double>("lidar_up_angle_deg", 15.0);
    this->declare_parameter<int>("sector_azimuth_steps", 120);

    this->declare_parameter<bool>("debug", true);
    this->declare_parameter<int>("debug_every_n_frames", 1);
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
    ego_speed_stale_timeout_sec_ =
      this->get_parameter("ego_speed_stale_timeout_sec").as_double();

    sensor_offset_x_ = this->get_parameter("sensor_offset_x").as_double();
    sensor_offset_y_ = this->get_parameter("sensor_offset_y").as_double();
    sensor_offset_z_ = this->get_parameter("sensor_offset_z").as_double();

    emergency_distance_ = this->get_parameter("emergency_distance").as_double();
    hazard_distance_ = this->get_parameter("hazard_distance").as_double();

    enable_side_zones_ = this->get_parameter("enable_side_zones").as_bool();
    side_zone_length_ = this->get_parameter("side_zone_length").as_double();
    side_zone_width_ = this->get_parameter("side_zone_width").as_double();
    side_zone_offset_y_ = this->get_parameter("side_zone_offset_y").as_double();

    min_track_hits_ = this->get_parameter("min_track_hits").as_int();
    max_track_misses_ = this->get_parameter("max_track_misses").as_int();

    emergency_hold_sec_ = this->get_parameter("emergency_hold_sec").as_double();
    hazard_hold_sec_ = this->get_parameter("hazard_hold_sec").as_double();

    marker_topic_ = this->get_parameter("marker_topic").as_string();
    marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();

    zone_height_ = this->get_parameter("zone_height").as_double();
    lidar_down_angle_deg_ = this->get_parameter("lidar_down_angle_deg").as_double();
    lidar_up_angle_deg_ = this->get_parameter("lidar_up_angle_deg").as_double();
    sector_azimuth_steps_ = this->get_parameter("sector_azimuth_steps").as_int();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ = this->get_parameter("debug_every_n_frames").as_int();

    if (qos_depth_ < 1) {
      qos_depth_ = 1;
    }

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 20.0;
    }

    stale_timeout_sec_ = std::max(0.0, stale_timeout_sec_);
    ego_speed_stale_timeout_sec_ = std::max(0.0, ego_speed_stale_timeout_sec_);

    emergency_distance_ = std::max(0.0, emergency_distance_);

    if (hazard_distance_ < emergency_distance_) {
      hazard_distance_ = emergency_distance_;
    }

    side_zone_length_ = std::max(0.0, side_zone_length_);
    side_zone_width_ = std::max(0.0, side_zone_width_);
    side_zone_offset_y_ = std::max(0.0, side_zone_offset_y_);

    min_track_hits_ = std::max(1, min_track_hits_);
    max_track_misses_ = std::max(0, max_track_misses_);

    emergency_hold_sec_ = std::max(0.0, emergency_hold_sec_);
    hazard_hold_sec_ = std::max(0.0, hazard_hold_sec_);

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

  // ==========================================================================
  // CALLBACKS
  // ==========================================================================
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

  // ==========================================================================
  // PARSE TRACKS
  // Supports stride 13 and stride 19.
  // ==========================================================================
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

  // ==========================================================================
  // MATH / FRAME HELPERS
  // ==========================================================================
  double degToRad(double deg) const
  {
    constexpr double pi = 3.14159265358979323846;
    return deg * pi / 180.0;
  }

  double frontZoneOriginX() const
  {
    // Front emergency/hazard is 180 graden vanuit de sensor/rosbag-origin.
    // De sensor staat in de rosbag op x=0.
    return 0.0;
  }

  double sideZoneFrontX() const
  {
    // Side zones starten bij x=0 en lopen naar achteren langs truck/trailer.
    return 0.0;
  }

  double truckFrontX() const
  {
    // Informatief:
    // Bij sensor_offset_x_ = -0.57 ligt de fysieke truckvoorkant in het
    // sensor/rosbag-frame op x=+0.57.
    return -sensor_offset_x_;
  }

  void toDetectionFrame(const TrackInput & t, double & x_d, double & y_d, double & z_d) const
  {
    // x/y blijven in het LiDAR/rosbag-frame.
    // Dus sensor-origin blijft x=0, y=0.
    x_d = static_cast<double>(t.x) +cd sensor_offset_x_;
    y_d = static_cast<double>(t.y) + sensor_offset_y_;

    // z wordt naar grondreferentie gebracht:
    // object-z t.o.v. sensor + sensorhoogte.
    z_d = static_cast<double>(t.z) + sensor_offset_z_;
  }

  double horizontalRangeFromSensor(double x_detection, double y_detection) const
  {
    // In het LiDAR/rosbag-frame staat de sensor zelf op x=0.
    return std::hypot(x_detection, y_detection - sensor_offset_y_);
  }

  double lowerVisibleHeightAtXY(double x_detection, double y_detection) const
  {
    const double angle_rad = degToRad(lidar_down_angle_deg_);
    const double range_xy = horizontalRangeFromSensor(x_detection, y_detection);
    const double z = sensor_offset_z_ - std::tan(angle_rad) * range_xy;
    return std::clamp(z, 0.0, zone_height_);
  }

  double upperVisibleHeightAtXY(double x_detection, double y_detection) const
  {
    const double angle_rad = degToRad(lidar_up_angle_deg_);
    const double range_xy = horizontalRangeFromSensor(x_detection, y_detection);
    const double z = sensor_offset_z_ + std::tan(angle_rad) * range_xy;
    return std::clamp(z, 0.0, zone_height_);
  }

  bool verticalIntervalIntersectsVlp16Frustum(
    double x_detection,
    double y_detection,
    double object_bottom_z,
    double object_top_z) const
  {
    const double low = lowerVisibleHeightAtXY(x_detection, y_detection);
    const double high = upperVisibleHeightAtXY(x_detection, y_detection);

    // Gedeeltelijke overlap is voldoende.
    // Object-interval [bottom, top] hoeft het LiDAR-zichtinterval [low, high]
    // alleen maar te raken.
    const double bottom = std::min(object_bottom_z, object_top_z);
    const double top = std::max(object_bottom_z, object_top_z);

    return top >= low && bottom <= high;
  }

  // ==========================================================================
  // DETECTION CLASSIFICATION
  // Detection geometry == RViz geometry.
  // ==========================================================================
  ZoneLevel classifyPoint3D(
    double x,
    double y,
    double object_bottom_z,
    double object_top_z) const
  {
    if (!verticalIntervalIntersectsVlp16Frustum(
          x,
          y,
          object_bottom_z,
          object_top_z))
    {
      return ZoneLevel::Free;
    }

    const double x_rel_front = x - frontZoneOriginX();
    const double y_rel_front = y;

    const double r = std::hypot(x_rel_front, y_rel_front);
    const double angle = std::atan2(y_rel_front, x_rel_front);

    // Frontsector:
    // 180 graden voor de sensor, dus x >= 0 in het LiDAR/rosbag-frame.
    const bool in_front_sector =
      (angle >= -degToRad(90.0)) &&
      (angle <= degToRad(90.0)) &&
      (x_rel_front >= 0.0);

    if (in_front_sector) {
      if (r <= emergency_distance_) {
        return ZoneLevel::Emergency;
      }

      if (emergency_distance_ <= r && r <= hazard_distance_) {
        return ZoneLevel::Hazard;
      }
    }

    // Side zones:
    // Alleen hazard. Rechthoeken links/rechts langs truck/trailer.
    // Start bij x=0 en loopt naar achteren.
    if (enable_side_zones_) {
      const double side_front_x = sideZoneFrontX();

      const bool in_side_x =
        (x <= side_front_x) &&
        (x >= side_front_x - side_zone_length_);

      if (in_side_x) {
        const double half = side_zone_width_ / 2.0;

        const bool left_hazard =
          (y >= side_zone_offset_y_ - half) &&
          (y <= side_zone_offset_y_ + half);

        const bool right_hazard =
          (y >= -side_zone_offset_y_ - half) &&
          (y <= -side_zone_offset_y_ + half);

        if (left_hazard || right_hazard) {
          return ZoneLevel::Hazard;
        }
      }
    }

    return ZoneLevel::Free;
  }

  ZoneLevel classifyObject(const TrackInput & t) const
  {
    const double half_x = 0.5 * std::max(0.0f, t.dx);
    const double half_y = 0.5 * std::max(0.0f, t.dy);
    const double half_z = 0.5 * std::max(0.0f, t.dz);

    double x_d = 0.0;
    double y_d = 0.0;
    double z_d = 0.0;

    toDetectionFrame(t, x_d, y_d, z_d);

    const double x_min = x_d - half_x;
    const double x_max = x_d + half_x;
    const double y_min = y_d - half_y;
    const double y_max = y_d + half_y;

    const double object_bottom_z = z_d - half_z;
    const double object_top_z = z_d + half_z;

    std::vector<std::pair<double, double>> samples;
    samples.reserve(9);

    // 3x3 raster over de bounding box.
    // Hierdoor triggert een object ook als alleen een corner in de zone valt.
    const double xs[3] = {x_min, x_d, x_max};
    const double ys[3] = {y_min, y_d, y_max};

    for (double xx : xs) {
      for (double yy : ys) {
        samples.emplace_back(xx, yy);
      }
    }

    ZoneLevel best = ZoneLevel::Free;

    for (const auto & sample : samples) {
      const ZoneLevel zone =
        classifyPoint3D(
          sample.first,
          sample.second,
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

  int computeSafety()
  {
    int signal = 2;

    for (const auto & t : tracks_) {
      if (t.hits < min_track_hits_) {
        continue;
      }

      if (t.misses > max_track_misses_) {
        continue;
      }

      const ZoneLevel zone = classifyObject(t);

      if (zone == ZoneLevel::Emergency) {
        return 0;
      }

      if (zone == ZoneLevel::Hazard) {
        signal = 1;
      }
    }

    return signal;
  }

  // ==========================================================================
  // VISUALISATION HELPERS
  // ==========================================================================
  geometry_msgs::msg::Point point(double x, double y, double z) const
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  void addTwoSidedTriangle(
    std::vector<geometry_msgs::msg::Point> & points,
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b,
    const geometry_msgs::msg::Point & c) const
  {
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);

    points.push_back(c);
    points.push_back(b);
    points.push_back(a);
  }

  visualization_msgs::msg::Marker makeSectorFrustum(
    int id,
    double r_min,
    double r_max,
    float red,
    float green,
    float blue,
    float alpha) const
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = this->now();

    marker.ns = "zones";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    const int steps = std::max(6, sector_azimuth_steps_);
    const double angle_min = -degToRad(90.0);
    const double angle_max = degToRad(90.0);
    const double dtheta = (angle_max - angle_min) / static_cast<double>(steps);

    for (int i = 0; i < steps; ++i) {
      const double a0 = angle_min + static_cast<double>(i) * dtheta;
      const double a1 = angle_min + static_cast<double>(i + 1) * dtheta;

      const double front_x = frontZoneOriginX();

      const double x0i = front_x + r_min * std::cos(a0);
      const double y0i = r_min * std::sin(a0);
      const double x1i = front_x + r_min * std::cos(a1);
      const double y1i = r_min * std::sin(a1);

      const double x0o = front_x + r_max * std::cos(a0);
      const double y0o = r_max * std::sin(a0);
      const double x1o = front_x + r_max * std::cos(a1);
      const double y1o = r_max * std::sin(a1);

      const auto A0 = point(x0i, y0i, lowerVisibleHeightAtXY(x0i, y0i));
      const auto B0 = point(x0o, y0o, lowerVisibleHeightAtXY(x0o, y0o));
      const auto C0 = point(x1o, y1o, lowerVisibleHeightAtXY(x1o, y1o));
      const auto D0 = point(x1i, y1i, lowerVisibleHeightAtXY(x1i, y1i));

      const auto A1 = point(x0i, y0i, upperVisibleHeightAtXY(x0i, y0i));
      const auto B1 = point(x0o, y0o, upperVisibleHeightAtXY(x0o, y0o));
      const auto C1 = point(x1o, y1o, upperVisibleHeightAtXY(x1o, y1o));
      const auto D1 = point(x1i, y1i, upperVisibleHeightAtXY(x1i, y1i));

      // Onderkant
      addTwoSidedTriangle(marker.points, A0, B0, C0);
      addTwoSidedTriangle(marker.points, A0, C0, D0);

      // Bovenkant
      addTwoSidedTriangle(marker.points, A1, C1, B1);
      addTwoSidedTriangle(marker.points, A1, D1, C1);

      // Buitenboog
      addTwoSidedTriangle(marker.points, B0, B1, C1);
      addTwoSidedTriangle(marker.points, B0, C1, C0);

      // Binnenboog, alleen bij hazard-ring.
      if (r_min > 1e-6) {
        addTwoSidedTriangle(marker.points, A0, D1, A1);
        addTwoSidedTriangle(marker.points, A0, D0, D1);
      }
    }

    return marker;
  }

  visualization_msgs::msg::Marker makeSideHazardFrustum(
    int id,
    double y_center) const
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = this->now();

    marker.ns = "zones";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.25f;

    const double side_front_x = sideZoneFrontX();
    const double x0 = side_front_x - side_zone_length_;
    const double x1 = side_front_x;

    const double half = side_zone_width_ / 2.0;
    const double y0 = y_center - half;
    const double y1 = y_center + half;

    const auto A0 = point(x0, y0, lowerVisibleHeightAtXY(x0, y0));
    const auto B0 = point(x0, y1, lowerVisibleHeightAtXY(x0, y1));
    const auto C0 = point(x1, y1, lowerVisibleHeightAtXY(x1, y1));
    const auto D0 = point(x1, y0, lowerVisibleHeightAtXY(x1, y0));

    const auto A1 = point(x0, y0, upperVisibleHeightAtXY(x0, y0));
    const auto B1 = point(x0, y1, upperVisibleHeightAtXY(x0, y1));
    const auto C1 = point(x1, y1, upperVisibleHeightAtXY(x1, y1));
    const auto D1 = point(x1, y0, upperVisibleHeightAtXY(x1, y0));

    // Onderkant
    addTwoSidedTriangle(marker.points, A0, B0, C0);
    addTwoSidedTriangle(marker.points, A0, C0, D0);

    // Bovenkant
    addTwoSidedTriangle(marker.points, A1, C1, B1);
    addTwoSidedTriangle(marker.points, A1, D1, C1);

    // Zijkanten
    addTwoSidedTriangle(marker.points, A0, A1, B1);
    addTwoSidedTriangle(marker.points, A0, B1, B0);

    addTwoSidedTriangle(marker.points, D0, C0, C1);
    addTwoSidedTriangle(marker.points, D0, C1, D1);

    // Kopse kanten
    addTwoSidedTriangle(marker.points, A0, D0, D1);
    addTwoSidedTriangle(marker.points, A0, D1, A1);

    addTwoSidedTriangle(marker.points, B0, B1, C1);
    addTwoSidedTriangle(marker.points, B0, C1, C0);

    return marker;
  }

  // ==========================================================================
  // PUBLISH ZONES
  // ==========================================================================
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

    // Emergency: rood, 0 -> emergency_distance
    arr.markers.push_back(
      makeSectorFrustum(
        1,
        0.0,
        emergency_distance_,
        1.0f,
        0.0f,
        0.0f,
        0.35f));

    // Hazard: oranje, emergency_distance -> hazard_distance
    arr.markers.push_back(
      makeSectorFrustum(
        2,
        emergency_distance_,
        hazard_distance_,
        1.0f,
        0.5f,
        0.0f,
        0.25f));

    // Side hazard zones: oranje rechthoeken links/rechts
    if (enable_side_zones_) {
      arr.markers.push_back(
        makeSideHazardFrustum(
          3,
          side_zone_offset_y_));

      arr.markers.push_back(
        makeSideHazardFrustum(
          4,
          -side_zone_offset_y_));
    }

    marker_pub_->publish(arr);
  }

  // ==========================================================================
  // STATUS MARKER
  // ==========================================================================
  void publishStatusMarker(int signal)
  {
    visualization_msgs::msg::Marker bg;
    bg.header.frame_id = marker_frame_id_;
    bg.header.stamp = this->now();
    bg.ns = "safety_status";
    bg.id = 100;
    bg.type = visualization_msgs::msg::Marker::CUBE;
    bg.action = visualization_msgs::msg::Marker::ADD;

    bg.scale.x = 2.0;
    bg.scale.y = 0.1;
    bg.scale.z = 0.5;
    bg.pose.position.x = frontZoneOriginX() + 1.0;
    bg.pose.position.y = 0.0;
    bg.pose.position.z = 2.0;

    bg.color.r = 0.0f;
    bg.color.g = 0.0f;
    bg.color.b = 0.0f;
    bg.color.a = 0.8f;

    status_marker_pub_->publish(bg);

    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = marker_frame_id_;
    txt.header.stamp = this->now();
    txt.ns = "safety_status";
    txt.id = 101;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;

    txt.scale.z = 0.35;
    txt.pose.position.x = frontZoneOriginX() + 1.0;
    txt.pose.position.y = 0.0;
    txt.pose.position.z = 2.0;

    if (signal == 0) {
      txt.text = "STOP";
      txt.color.r = 1.0f;
      txt.color.g = 0.0f;
      txt.color.b = 0.0f;
      txt.color.a = 1.0f;
    } else if (signal == 1) {
      txt.text = "WARNING";
      txt.color.r = 1.0f;
      txt.color.g = 0.5f;
      txt.color.b = 0.0f;
      txt.color.a = 1.0f;
    } else {
      txt.text = "FREE";
      txt.color.r = 0.0f;
      txt.color.g = 1.0f;
      txt.color.b = 0.0f;
      txt.color.a = 1.0f;
    }

    status_marker_pub_->publish(txt);
  }

  // ==========================================================================
  // TIMER
  // ==========================================================================
  void timerCallback()
  {
    const double tracks_age = (this->now() - last_tracks_time_).seconds();

    if (tracks_age > stale_timeout_sec_) {
      std_msgs::msg::Int16 stale_msg;
      stale_msg.data = static_cast<int16_t>(stale_signal_);
      pub_->publish(stale_msg);
      publishZones();
      publishStatusMarker(stale_msg.data);
      return;
    }

    const int raw_signal = computeSafety();
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

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(latched_signal_);
    pub_->publish(msg);

    publishStatusMarker(latched_signal_);
    publishZones();

    if (debug_ && (frame_count_ % debug_every_n_frames_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "raw_signal=%d latched_signal=%d tracks=%zu",
        raw_signal,
        latched_signal_,
        tracks_.size());
    }
  }

  // ==========================================================================
  // MEMBERS
  // ==========================================================================
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

  double emergency_distance_ = 1.5;
  double hazard_distance_ = 2.5;

  bool enable_side_zones_ = true;
  double side_zone_length_ = 2.0;
  double side_zone_width_ = 0.8;
  double side_zone_offset_y_ = 0.8;

  int min_track_hits_ = 1;
  int max_track_misses_ = 2;


  double emergency_hold_sec_ = 0.25;
  double hazard_hold_sec_ = 0.25;

  double zone_height_ = 3.0;
  double lidar_down_angle_deg_ = 15.0;
  double lidar_up_angle_deg_ = 15.0;

  int sector_azimuth_steps_ = 120;

  bool debug_ = true;
  int debug_every_n_frames_ = 1;

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}